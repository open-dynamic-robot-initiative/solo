#include "solo/solo12.hpp"
#include <cmath>
#include <odri_control_interface/common.hpp>
#include "solo/common_programs_header.hpp"
#include "real_time_tools/spinner.hpp"
#include "odri_control_interface/utils.hpp"

namespace solo
{
const double Solo12::max_joint_torque_security_margin_ = 0.99;

using namespace odri_control_interface;

Solo12::Solo12()
{
    /**
     * Hardware properties
     */
    motor_inertias_.setZero();
    motor_torque_constants_.setZero();
    joint_gear_ratios_.setZero();
    motor_max_current_.setZero();
    max_joint_torques_.setZero();
    joint_zero_positions_.setZero();
    slider_positions_vector_.resize(5);

    /**
     * Hardware status
     */
    for (unsigned i = 0; i < motor_enabled_.size(); ++i)
    {
        motor_enabled_[i] = false;
        motor_ready_[i] = false;
    }
    for (unsigned i = 0; i < motor_board_enabled_.size(); ++i)
    {
        motor_board_enabled_[0] = false;
        motor_board_errors_[0] = 0;
    }

    /**
     * Joint data
     */
    joint_positions_.setZero();
    joint_velocities_.setZero();
    joint_torques_.setZero();
    joint_target_torques_.setZero();
    joint_encoder_index_.setZero();

    /**
     * Additional data
     */
    slider_positions_.setZero();
    contact_sensors_states_.setZero();
    imu_accelerometer_.setZero();
    imu_gyroscope_.setZero();
    imu_attitude_.setZero();
    imu_linear_acceleration_.setZero();
    imu_attitude_quaternion_.setIdentity();

    /**
     * Setup some known data
     */

    // for now this value is very small but it is currently for debug mode
    motor_max_current_.fill(8.0);  // TODO: set as paramters?
    motor_torque_constants_.fill(0.025);
    motor_inertias_.fill(0.045);
    joint_gear_ratios_.fill(9.0);

    // By default assume the estop is inactive.
    active_estop_ = false;
    calibrate_request_ = false;

    state_ = Solo12State::initial;
}

void Solo12::initialize(const std::string& network_id,
                        const std::string& serial_port)
{
    network_id_ = network_id;

    // Use a serial port to read slider values.
    serial_reader_ =
        std::make_shared<blmc_drivers::SerialReader>(serial_port, 5);

    // Main driver interface.
    robot_ = odri_control_interface::RobotFromYamlFile(
        network_id_, ODRI_CONTROL_INTERFACE_YAML_PATH);

    calib_ctrl_ = odri_control_interface::JointCalibratorFromYamlFile(
        ODRI_CONTROL_INTERFACE_YAML_PATH, robot_->joints);

    // Initialize the robot.
    robot_->Init();
}

void Solo12::acquire_sensors()
{
    static int estop_counter_ = 0;

    robot_->ParseSensorData();

    auto joints = robot_->joints;
    auto imu = robot_->imu;

    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = joints->GetPositions();
    // acquire the joint velocities
    joint_velocities_ = joints->GetVelocities();
    // acquire the joint torques
    joint_torques_ = joints->GetMeasuredTorques();
    // acquire the target joint torques
    joint_target_torques_ = joints->GetSentTorques();

    // TODO: The index angle is not transmitted.
    // joint_encoder_index_ = joints_.get_measured_index_angles();

    /**
     * Additional data
     */
    // acquire the slider positions
    // TODO: Handle case that no new values are arriving.
    serial_reader_->fill_vector(slider_positions_vector_);
    for (unsigned i = 0; i < slider_positions_.size(); ++i)
    {
        // acquire the slider
        slider_positions_(i) = double(slider_positions_vector_[i + 1]) / 1024.;
    }

    // Active the estop if button is pressed or the estop was active before.
    active_estop_ |= slider_positions_vector_[0] == 0;

    if (active_estop_ && estop_counter_++ % 2000 == 0)
    {
        robot_->ReportError("Soft E-Stop is active.");
    }

    // acquire imu
    imu_linear_acceleration_ = imu->GetLinearAcceleration();
    imu_accelerometer_ = imu->GetAccelerometer();
    imu_gyroscope_ = imu->GetGyroscope();
    imu_attitude_ = imu->GetAttitudeEuler();
    imu_attitude_quaternion_ = imu->GetAttitudeQuaternion();

    /**
     * The different status.
     */

    // motor board status
    ConstRefVectorXi motor_board_errors = joints->GetMotorDriverErrors();
    ConstRefVectorXb motor_driver_enabled = joints->GetMotorDriverEnabled();
    for (int i = 0; i < 6; i++)
    {
        motor_board_errors_[i] = motor_board_errors[i];
        motor_board_enabled_[i] = motor_driver_enabled[i];
    }

    // motors status
    ConstRefVectorXb motor_enabled = joints->GetEnabled();
    ConstRefVectorXb motor_ready = joints->GetReady();
    for (int i = 0; i < 12; i++)
    {
        motor_enabled_[i] = motor_enabled[i];
        motor_ready_[i] = motor_ready[i];
    }
}

void Solo12::set_max_current(const double& max_current)
{
    robot_->joints->SetMaximumCurrents(max_current);
}

void Solo12::send_target_joint_torque(
    const Eigen::Ref<Vector12d> target_joint_torque)
{
    robot_->joints->SetTorques(target_joint_torque);

    switch (state_)
    {
        case Solo12State::initial:
            robot_->joints->SetZeroCommands();
            if (!robot_->IsTimeout() && !robot_->IsAckMsgReceived())
            {
                robot_->SendInit();
            }
            else if (!robot_->IsReady())
            {
                robot_->SendCommand();
            }
            else
            {
                state_ = Solo12State::ready;
            }
            break;

        case Solo12State::ready:
            if (calibrate_request_)
            {
                calibrate_request_ = false;
                state_ = Solo12State::calibrate;
                _is_calibrating = true;
                robot_->joints->SetZeroCommands();
            }
            robot_->SendCommand();
            break;

        case Solo12State::calibrate:
            if (calib_ctrl_->Run())
            {
                state_ = Solo12State::ready;
                _is_calibrating = false;
            }
            robot_->SendCommand();
            break;
    }
}

void Solo12::wait_until_ready()
{
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    static long int count_wait_until_ready = 0;
    while(state_ != Solo12State::ready)
    {
        if (count_wait_until_ready % 200 == 0)
        {
            printf("Solo12::wait_until_ready Getting ready\n");
        }
        spinner.spin();
        count_wait_until_ready++;
    }
}

bool Solo12::is_ready()
{
    return state_ != Solo12State::ready;
}

bool Solo12::request_calibration(const Vector12d& home_offset_rad)
{
    printf("Solo12::request_calibration called\n");
    Eigen::VectorXd hor = home_offset_rad;
    calib_ctrl_->UpdatePositionOffsets(hor);
    calibrate_request_ = true;
    return true;
}

}  // namespace solo
