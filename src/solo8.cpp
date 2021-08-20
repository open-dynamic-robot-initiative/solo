#include "solo/solo8.hpp"
#include <cmath>
#include "solo/common_programs_header.hpp"
#include <odri_control_interface/common.hpp>

namespace solo
{
const double Solo8::max_joint_torque_security_margin_ = 0.99;

using namespace odri_control_interface;

Solo8::Solo8()
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

    /**
     * Setup some known data
     */

    // for now this value is very small but it is currently for debug mode
    motor_max_current_.fill(4.0);  // TODO: set as paramters?
    motor_torque_constants_.fill(0.025);
    motor_inertias_.fill(0.045);
    joint_gear_ratios_.fill(9.0);

    slider_positions_vector_.resize(3);
    active_estop_ = false;

    state_ = Solo8State::initial;
}

void Solo8::initialize(const std::string& network_id)
{
    // Use a serial port to read slider values.
    serial_reader_ =
        std::make_shared<slider_box::SerialReader>("Not used", 3);

    main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id);

    VectorXi motor_numbers(8);
    motor_numbers << 0, 1, 3, 2, 5, 4, 6, 7;
    VectorXb motor_reversed(8);
    motor_reversed << true, true, false, false, true, true, false, false;

    double lHFE = 1.45;
    double lKFE = 2.80;
    Eigen::VectorXd joint_lower_limits(8);
    joint_lower_limits << -lHFE, -lKFE, -lHFE, -lKFE, -lHFE, -lKFE, -lHFE, -lKFE;
    Eigen::VectorXd joint_upper_limits(8);
    joint_upper_limits << lHFE, lKFE, lHFE, lKFE, lHFE, lKFE, lHFE, lKFE;

    // Define the joint module.
    joints_ = std::make_shared<odri_control_interface::JointModules>(
        main_board_ptr_,
        motor_numbers,
        motor_torque_constants_(0),
        joint_gear_ratios_(0),
        motor_max_current_(0),
        motor_reversed,
        joint_lower_limits,
        joint_upper_limits,
        80.,
        0.2);

    // Define the IMU.
    VectorXl rotate_vector(3);
    rotate_vector << 1, 2, 3;
    VectorXl orientation_vector(4);
    orientation_vector << 1, 2, 3, 4;
    imu_ = std::make_shared<odri_control_interface::IMU>(
        main_board_ptr_, rotate_vector, orientation_vector);

    // Use zero position offsets for now. Gets updated in the calibration
    // method.
    Eigen::VectorXd position_offsets(8);
    position_offsets.fill(0.);
    std::vector<odri_control_interface::CalibrationMethod> directions{
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE
       };
    calib_ctrl_ = std::make_shared<odri_control_interface::JointCalibrator>(
        robot_->joints, directions, position_offsets, 5., 0.05, 1.0, 0.001);

    // Define the robot.
    robot_ = std::make_shared<odri_control_interface::Robot>(
        main_board_ptr_, joints_, imu_, calib_ctrl_);

    // Initialize the robot.
    robot_->Init();
}

void Solo8::acquire_sensors()
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
    for (int i = 0; i < 4; i++)
    {
        motor_board_errors_[i] = motor_board_errors[i];
        motor_board_enabled_[i] = motor_driver_enabled[i];
    }

    // motors status
    ConstRefVectorXb motor_enabled = joints->GetEnabled();
    ConstRefVectorXb motor_ready = joints->GetReady();
    for (int i = 0; i < 8; i++)
    {
        motor_enabled_[i] = motor_enabled[i];
        motor_ready_[i] = motor_ready[i];
    }
}

void Solo8::send_target_joint_torque(
    const Eigen::Ref<Vector8d> target_joint_torque)
{
    robot_->joints->SetTorques(target_joint_torque);

    switch (state_)
    {
        case Solo8State::initial:
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
                state_ = Solo8State::ready;
            }
            break;

        case Solo8State::ready:
            if (calibrate_request_)
            {
                calibrate_request_ = false;
                state_ = Solo8State::calibrate;
                _is_calibrating = true;
                robot_->joints->SetZeroCommands();
            }
            robot_->SendCommand();
            break;

        case Solo8State::calibrate:
            if (calib_ctrl_->Run())
            {
                state_ = Solo8State::ready;
                _is_calibrating = false;
            }
            robot_->SendCommand();
            break;
    }
}

bool Solo8::request_calibration(const Vector8d& home_offset_rad)
{
    printf("Solo8::request_calibration called\n");
    Eigen::VectorXd hor = home_offset_rad;
    calib_ctrl_->UpdatePositionOffsets(hor);
    calibrate_request_ = true;
    return true;
}

}  // namespace solo
