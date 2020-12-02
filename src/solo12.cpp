#include "blmc_robots/solo12.hpp"
#include <cmath>
#include "blmc_robots/common_programs_header.hpp"
#include "real_time_tools/spinner.hpp"


namespace blmc_robots
{
const double Solo12::max_joint_torque_security_margin_ = 0.99;

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
    reverse_polarities_.fill(false);
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
    active_estop_= false;
}

void Solo12::initialize(const std::string& network_id,
                        const std::string& serial_port)
{
    network_id_ = network_id;

    // Use a serial port to read slider values.
    serial_reader_ =
        std::make_shared<blmc_drivers::SerialReader>(serial_port, 5);

    auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id_);

    std::array<int, 12> motor_numbers = {
        0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10};
    std::array<bool, 12> motor_reversed = {
        true, false, true, true, false, false,
        true, false, true, true, false, false};

    std::array<double, 12> joint_lower_limits = {
            -1.2, -1.7, -3.4, -1.2, -1.7, -3.4,
            -1.2, -1.7, -3.4, -1.2, -1.7, -3.4
    };
    std::array<double, 12> joint_upper_limits = {
            1.2,  1.7, +3.4, +1.2, +1.7, +3.4,
            1.2,  1.7, +3.4, +1.2, +1.7, +3.4
    };

    // Define the joint module.
    auto joints = std::make_shared<odri_control_interface::JointModules<12> >(
        main_board_ptr_,
        motor_numbers,
        0.025, 9., 1.,
        motor_reversed,
        joint_lower_limits, joint_upper_limits, 80., 0.5
    );

    // Define the IMU.
    std::array<int, 3> rotate_vector = {1, 2, 3};
    std::array<int, 4> orientation_vector = {1, 2, 3, 4};
    auto imu = std::make_shared<odri_control_interface::IMU>(main_board_ptr_,
        rotate_vector, orientation_vector);

    // Define the robot.
    robot_ = std::make_shared<odri_control_interface::Robot<12> >(
        main_board_ptr_, joints, imu
    );

    // Start the robot.
    robot_->Start();
    robot_->WaitUntilReady();

    rt_printf("All motors and boards are ready.\n");
}

#define Map3(arr) Eigen::Map<Eigen::Vector3d>(arr.data())
#define Map4(arr) Eigen::Map<Eigen::Vector4d>(arr.data())
#define Map12(arr) Eigen::Map<Vector12d>(arr.data())

void Solo12::acquire_sensors()
{
    robot_->ParseSensorData();

    auto joints = robot_->joints;
    auto imu = robot_->imu;

    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = Map12(joints->GetPositions());
    // acquire the joint velocities
    joint_velocities_ = Map12(joints->GetVelocities());
    // acquire the joint torques
    joint_torques_ = Map12(joints->GetMeasuredTorques());
    // acquire the target joint torques
    joint_target_torques_ = Map12(joints->GetSentTorques());

    // TODO.
    // The index angle is not transmitted.
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

    // acquire imu
    imu_accelerometer_ = Map3(imu->GetAccelerometer());
    imu_gyroscope_ = Map3(imu->GetGyroscope());
    imu_attitude_ = Map3(imu->GetAttitudeEuler());
    imu_attitude_quaternion_ = Map4(imu->GetAttitudeQuaternion());

    /**
     * The different status.
     */

    // motor board status
    motor_board_enabled_ = joints->GetMotorDriverEnabled();
    motor_board_errors_ = joints->GetMotorDriverErrors();

    // motors status
    motor_enabled_ = joints->GetEnabled();
    motor_ready_ = joints->GetReady();
}

void Solo12::set_max_current(const double& max_current)
{
    robot_->joints->SetMaximumCurrents(max_current);
}

void Solo12::send_target_joint_torque(
    const Eigen::Ref<Vector12d> target_joint_torque)
{
    std::array<double, 12> torques;
    for (int i = 0; i < 12; i++)
    {
        torques[i] = target_joint_torque(i);
    }
    robot_->joints->SetTorques(torques);
    robot_->SendCommand();
}

bool Solo12::calibrate(const Vector12d& home_offset_rad)
{
    std::array<odri_control_interface::CalibrationMethod, 12> directions = {
        odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE, odri_control_interface::POSITIVE
    };
    std::array<double, 12> position_offsets;
    for (int i = 0; i < 12; i++) {
        position_offsets[i] = home_offset_rad(i);
    }
    auto calib_ctrl = std::make_shared<odri_control_interface::JointCalibrator<12> >(
        robot_->joints,
        directions,
        position_offsets,
        5., 0.05, 0.001
    );

    bool calibration_done = false;
    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
    while (!robot_->HasError() && !calibration_done)
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
        {
            last += std::chrono::milliseconds(1);
            robot_->ParseSensorData();
            calibration_done |= calib_ctrl->Run();
            robot_->SendCommand();
        } else {
            std::this_thread::yield();
        }
    }

    return true;
}

}  // namespace blmc_robots
