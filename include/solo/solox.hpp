/**
 * @file
 * @brief Interface for Solo robots using master board.
 * @copyright Copyright (c) 2019 New York University & Max Planck Gesellschaft
 */
#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/error.hpp>
#include <odri_control_interface/robot.hpp>
#include <slider_box/serial_reader.hpp>

#include "common_header.hpp"

namespace solo
{
enum class SoloState
{
    initial,
    ready,
    calibrate
};

/**
 * @brief Interface for the Solo robots using the master board.
 *
 * This class is templated with the number of joints and can be used for both
 * Solo8 and Solo12 by setting the corresponding number.
 *
 * Solo8
 * =====
 *
 * TODO
 *
 * Solo12
 * ======
 *
 * Mapping between the DOF and driver boards + motor ports:
 *
 * - FL_HAA: motor board 0, motor port 0, motor index 0
 * - FL_HFE: motor board 1, motor port 1, motor index 3
 * - FL_KFE: motor board 1, motor port 0, motor index 2
 * - FR_HAA: motor board 0, motor port 1, motor index 1
 * - FR_HFE: motor board 2, motor port 1, motor index 5
 * - FR_KFE: motor board 2, motor port 0, motor index 4
 * - HL_HAA: motor board 3, motor port 0, motor index 6
 * - HL_HFE: motor board 4, motor port 1, motor index 9
 * - HL_KFE: motor board 4, motor port 0, motor index 8
 * - HR_HAA: motor board 3, motor port 1, motor index 7
 * - HR_HFE: motor board 5, motor port 1, motor index 11
 * - HR_KFE: motor board 5, motor port 0, motor index 10

 *
 * @tparam X Number of joints of the robot.  Should be either 8 or 12.
 */
template <int X>
class SoloX
{
public:
    //! @brief Set slider box port to this value to disable it.
    inline static const std::string SLIDER_BOX_DISABLED = "none";

    //! @brief Name of the spdlog logger used by the class.
    inline static const std::string LOGGER_NAME = "solo/Solo";

    /**
     * @brief Solo is the constructor of the class.
     */
    SoloX();

    /**
     * @brief Initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0.
     *
     * @param network_id Name of the network interface for connection to the
     *      robot.
     * @param slider_box_port Name of the serial port to which the slider box is
     *      connected.  Set to "" or "none" if no slider box is used.  Set to
     *      "auto" to auto-detect the port.
     */
    void initialize(const std::string& network_id,
                    const std::string& slider_box_port = "auto");

    /**
     * @brief Sets the maximum joint torques.
     */
    void set_max_current(const double& max_current);

    /**
     * @brief Wait that the robot enters into the ready states.
     */
    void wait_until_ready();

    /**
     * @brief Check if the robot is ready.
     */
    bool is_ready();

    /**
     * @brief send_target_torques sends the target currents to the motors.
     */
    void send_target_joint_torque(
        const Eigen::Ref<Vectord<X>> target_joint_torque);

    /**
     * @brief Sets the desired joint position of the P controller running on
     * the card.
     */
    void send_target_joint_position(
        const Eigen::Ref<Vectord<X>> target_joint_position);

    /**
     * @brief Sets the desired joint velocity of the D controller running on
     * the card.
     */
    void send_target_joint_velocity(
        const Eigen::Ref<Vectord<X>> target_joint_velocity);

    /**
     * @brief Sets the desired joint position gain P for the P controller
     * running on the card.
     */
    void send_target_joint_position_gains(
        const Eigen::Ref<Vectord<X>> target_joint_position_gains);

    /**
     * @brief Sets the desired joint velocity gain D for the D controller
     * running on the card.
     */
    void send_target_joint_velocity_gains(
        const Eigen::Ref<Vectord<X>> target_joint_velocity_gains);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    void acquire_sensors();

    /**
     * @brief Asynchronously request for the calibration.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true in case of success.
     * @return false in case of failure.
     */
    bool request_calibration(const Vectord<X>& home_offset_rad);

    /**
     * Joint properties
     */

    /**
     * @brief get_motor_inertias in [kg m^2]
     * @return Motor inertias.
     */
    const Eigen::Ref<Vectord<X>> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants in []
     * @return Torque constants of each motor.
     */
    const Eigen::Ref<Vectord<X>> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return Joint gear ratios
     */
    const Eigen::Ref<Vectord<X>> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_max_torque
     * @return the max torque that has been hardcoded in the constructor of this
     * class. @todo Parametrize the maximum current via yaml or something else.
     */
    const Eigen::Ref<Vectord<X>> get_motor_max_current()
    {
        return motor_max_current_;
    }

    /**
     * Sensor Data
     */

    /**
     * @brief get_joint_positions
     * @return  the joint angle of each module
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vectord<X>> get_joint_positions()
    {
        return joint_positions_;
    }

    /**
     * @brief get_joint_velocities
     * @return the joint velocities
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vectord<X>> get_joint_velocities()
    {
        return joint_velocities_;
    }

    /**
     * @brief get_joint_torques
     * @return the joint torques
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vectord<X>> get_joint_torques()
    {
        return joint_torques_;
    }

    /**
     * @brief get_joint_torques
     * @return the target joint torques
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vectord<X>> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
     * @brief get_joint_encoder_index
     * @return the position of the index of the encoders a the motor level
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vectord<X>> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

    /*
     * Additional data
     */

    /**
     * @brief get_contact_sensors_states
     * @return the state of the contacts states
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector4d> get_contact_sensors_states()
    {
        return contact_sensors_states_;
    }

    /**
     * @brief get_slider_positions
     * @return the current sliders positions.
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector4d> get_slider_positions()
    {
        return slider_positions_;
    }

    /** @brief base accelerometer from imu.
     * @return
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector3d> get_imu_accelerometer()
    {
        return imu_accelerometer_;
    }

    /** @brief base gyroscope from imu.
     * @return
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector3d> get_imu_gyroscope()
    {
        return imu_gyroscope_;
    }

    /** @brief base attitude quaternion (ordered {x, y, z, w}) from imu.
     * @return
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector4d> get_imu_attitude()
    {
        return imu_attitude_;
    }

    /** @brief base linear acceleration from imu.
     * @return
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector3d> get_imu_linear_acceleration()
    {
        return imu_linear_acceleration_;
    }

    /*
     * Hardware Status
     */

    /**
     * @brief get_motor_enabled
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, X>& get_motor_enabled()
    {
        return motor_enabled_;
    }

    /**
     * @brief get_motor_ready
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, X>& get_motor_ready()
    {
        return motor_ready_;
    }

    /**
     * @brief get_motor_board_enabled
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<bool, X / 2>& get_motor_board_enabled()
    {
        return motor_board_enabled_;
    }

    /**
     * @brief get_motor_board_errors
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<int, X / 2>& get_motor_board_errors()
    {
        return motor_board_errors_;
    }

    /**
     * @brief has_error
     * @return Returns true if the robot hardware has an error, false otherwise.
     */
    bool has_error() const
    {
        return robot_->HasError();
    }

    std::optional<odri_control_interface::ErrorMessage> get_error()
    {
        return robot_->GetError();
    }

    std::string get_error_description() const
    {
        return robot_->GetErrorDescription();
    }

    /**
     * @brief is_calibrating()
     * @return Returns true if the calibration procedure is running right now.
     */
    bool is_calibrating()
    {
        return _is_calibrating;
    }

    /**
     * @brief Get total number of command packets sent to the robot.
     */
    int get_num_sent_command_packets() const
    {
        return main_board_ptr_->GetCmdSent();
    }

    /**
     * @brief Get number of lost command packets.
     */
    int get_num_lost_command_packets() const
    {
        return main_board_ptr_->GetCmdLost();
    }

    /**
     * @brief Get total number of sensor packets sent by the robot.
     */
    int get_num_sent_sensor_packets() const
    {
        return main_board_ptr_->GetSensorsSent();
    }

    /**
     * @brief Get number of lost sensor packets.
     */
    int get_num_lost_sensor_packets() const
    {
        return main_board_ptr_->GetSensorsLost();
    }

private:
    //! Logger
    std::shared_ptr<spdlog::logger> log_;

    // Number of values sent by the slider box through the serial port.
    static constexpr int SLIDER_BOX_NUM_VALUES = 5;

    /**
     * Joint properties
     */

    /** @brief Motors inertia. */
    Vectord<X> motor_inertias_;
    /** @brief DCM motor torque constants. */
    Vectord<X> motor_torque_constants_;
    /** @brief joint gear ratios (9). */
    Vectord<X> joint_gear_ratios_;
    /** @brief Max appliable current before the robot shutdown. */
    Vectord<X> motor_max_current_;
    /** @brief Offset to the theoretical "0" pose. */
    Vectord<X> joint_zero_positions_;
    /** @brief Max joint torques (Nm) */
    Eigen::Array<double, X, 1> max_joint_torques_;
    /** @brief  Security margin on the saturation of the control. */
    static const double max_joint_torque_security_margin_;

    /**
     * -------------------------------------------------------------------------
     * Hardware status
     */

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, X> motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, X> motor_ready_;

    /**
     * @brief This gives the status
     * (enabled/disabled of the onboard control cards).
     */
    std::array<bool, X / 2> motor_board_enabled_;

    /**
     * @brief This gives the status
     * (enabled/disabled of the onboard control cards).
     */
    std::array<int, X / 2> motor_board_errors_;

    /**
     * Joint data
     */

    /**
     * @brief joint_positions_
     */
    Vectord<X> joint_positions_;
    /**
     * @brief joint_velocities_
     */
    Vectord<X> joint_velocities_;
    /**
     * @brief joint_torques_
     */
    Vectord<X> joint_torques_;
    /**
     * @brief joint_target_torques_
     */
    Vectord<X> joint_target_torques_;
    /**
     * @brief joint_encoder_index_
     */
    Vectord<X> joint_encoder_index_;

    /**
     * -------------------------------------------------------------------------
     * Additional data
     */

    /**
     * @brief slider_positions_ is the position of the linear potentiometer.
     * Can be used as a joystick input.
     */
    Eigen::Vector4d slider_positions_;

    /**
     * @brief contact_sensors_ is contact sensors at each feet of teh quadruped.
     */
    Eigen::Vector4d contact_sensors_states_;

    /** @brief This is the name of the network: Left column in ifconfig output
     */
    std::string network_id_;

    /** @brief Map the joint id to the motor board id, @see Solo12 description.
     */
    std::array<int, X> map_joint_id_to_motor_board_id_;
    /** @brief Map the joint id to the motor port id, @see Solo12 description.
     */
    std::array<int, X> map_joint_id_to_motor_port_id_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_accelerometer_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_gyroscope_;

    /** @brief base accelerometer. */
    Eigen::Vector4d imu_attitude_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_linear_acceleration_;

    /** @brief State of the solo robot. */
    SoloState state_;

    /** @brief Controller to run the calibration procedure */
    std::shared_ptr<odri_control_interface::JointCalibrator> calib_ctrl_;

    /** @brief Indicator if calibration should start. */
    bool calibrate_request_;

    /**
     * Drivers communication objects
     */

    /**
     * @brief Main board drivers.
     *
     * PC <- Ethernet/Wifi -> main board <- SPI -> Motor Board
     */
    std::shared_ptr<MasterBoardInterface> main_board_ptr_;

    /**
     * @brief Reader for serial port to read arduino slider values.
     */
    std::shared_ptr<slider_box::SerialReader> serial_reader_;

    /**
     * @brief The odri robot abstraction.
     */
    std::shared_ptr<odri_control_interface::Robot> robot_;

    /**
     * @brief Collection of Joints.
     */
    std::shared_ptr<odri_control_interface::JointModules> joints_;

    /**
     * @brief Robot Imu drivers.
     */
    std::shared_ptr<odri_control_interface::IMU> imu_;

    /** @brief If the physical estop is pressed or not. */
    bool active_estop_;

    /** @brief If the joint calibration is active or not. */
    bool _is_calibrating;

    void initialize_joint_modules();
    std::vector<odri_control_interface::CalibrationMethod>
    get_calibration_directions() const;
};

}  // namespace solo

//
// Implementation
//

namespace solo
{
template <int X>
const double SoloX<X>::max_joint_torque_security_margin_ = 0.99;

template <int X>
SoloX<X>::SoloX()
{
    // initialise logger and set level
    log_ = spdlog::get(LOGGER_NAME);
    if (!log_)
    {
        log_ = spdlog::stderr_color_mt(LOGGER_NAME);
        log_->set_level(spdlog::level::debug);
    }

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
    imu_accelerometer_.setZero();
    imu_gyroscope_.setZero();
    imu_attitude_.setZero();
    imu_linear_acceleration_.setZero();

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

    state_ = SoloState::initial;
}

template <int X>
void SoloX<X>::initialize(const std::string& network_id,
                          const std::string& slider_box_port)
{
    network_id_ = network_id;

    // only initialize serial reader if
    if (!slider_box_port.empty() and slider_box_port != SLIDER_BOX_DISABLED)
    {
        log_->debug("Use slider box at port '{}'", slider_box_port);
        // Use a serial port to read slider values.
        serial_reader_ = std::make_shared<slider_box::SerialReader>(
            slider_box_port, SLIDER_BOX_NUM_VALUES);
    }
    else
    {
        log_->info("No slider box port provided.  Slider box is disabled.");
    }

    main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id_);

    initialize_joint_modules();

    // Define the IMU.
    odri_control_interface::VectorXl rotate_vector(3);
    rotate_vector << 1, 2, 3;
    odri_control_interface::VectorXl orientation_vector(4);
    orientation_vector << 1, 2, 3, 4;
    imu_ = std::make_shared<odri_control_interface::IMU>(
        main_board_ptr_, rotate_vector, orientation_vector);

    // Use zero position offsets for now. Gets updated in the calibration
    // method.
    Eigen::VectorXd position_offsets(X), calibration_position(X);
    Eigen::VectorXi calibration_order(X);
    position_offsets.fill(0.);
    calibration_position.fill(0.);
    calibration_order.fill(0);
    auto directions = get_calibration_directions();
    calib_ctrl_ = std::make_shared<odri_control_interface::JointCalibrator>(
        joints_,
        directions,
        position_offsets,
        calibration_order,
        calibration_position,
        5.,
        0.05,
        1.0,
        0.001);

    // Define the robot.
    robot_ = std::make_shared<odri_control_interface::Robot>(
        main_board_ptr_, joints_, imu_, calib_ctrl_);

    // Initialize the robot.
    robot_->Init();
}

template <int X>
void SoloX<X>::acquire_sensors()
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
    if (serial_reader_)
    {
        std::vector<int> slider_box_values(SLIDER_BOX_NUM_VALUES);

        // acquire the slider positions
        // TODO: Handle case that no new values are arriving.
        serial_reader_->fill_vector(slider_box_values);
        for (unsigned i = 0; i < slider_positions_.size(); ++i)
        {
            // acquire the slider
            slider_positions_(i) = double(slider_box_values[i + 1]) / 1024.;
        }

        // Active the estop if button is pressed or the estop was active before.
        active_estop_ |= slider_box_values[0] == 0;
    }

    if (active_estop_ && estop_counter_++ % 2000 == 0)
    {
        robot_->ReportError("Soft E-Stop is active.");
    }

    // acquire imu
    imu_linear_acceleration_ = imu->GetLinearAcceleration();
    imu_accelerometer_ = imu->GetAccelerometer();
    imu_gyroscope_ = imu->GetGyroscope();
    imu_attitude_ = imu->GetAttitudeQuaternion();

    /**
     * The different status.
     */

    // motor board status
    odri_control_interface::ConstRefVectorXi motor_board_errors =
        joints->GetMotorDriverErrors();
    odri_control_interface::ConstRefVectorXb motor_driver_enabled =
        joints->GetMotorDriverEnabled();
    for (int i = 0; i < 6; i++)
    {
        motor_board_errors_[i] = motor_board_errors[i];
        motor_board_enabled_[i] = motor_driver_enabled[i];
    }

    // motors status
    odri_control_interface::ConstRefVectorXb motor_enabled =
        joints->GetEnabled();
    odri_control_interface::ConstRefVectorXb motor_ready = joints->GetReady();
    for (int i = 0; i < 12; i++)
    {
        motor_enabled_[i] = motor_enabled[i];
        motor_ready_[i] = motor_ready[i];
    }
}

template <int X>
void SoloX<X>::set_max_current(const double& max_current)
{
    robot_->joints->SetMaximumCurrents(max_current);
}

template <int X>
void SoloX<X>::send_target_joint_torque(
    const Eigen::Ref<Vectord<X>> target_joint_torque)
{
    robot_->joints->SetTorques(target_joint_torque);

    switch (state_)
    {
        case SoloState::initial:
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
                state_ = SoloState::ready;
            }
            break;

        case SoloState::ready:
            if (calibrate_request_)
            {
                calibrate_request_ = false;
                state_ = SoloState::calibrate;
                _is_calibrating = true;
                robot_->joints->SetZeroCommands();
            }
            robot_->SendCommand();
            break;

        case SoloState::calibrate:
            if (calib_ctrl_->Run())
            {
                state_ = SoloState::ready;
                _is_calibrating = false;
            }
            robot_->SendCommand();
            break;
    }
}

template <int X>
void SoloX<X>::send_target_joint_position(
    const Eigen::Ref<Vectord<X>> target_joint_position)
{
    robot_->joints->SetDesiredPositions(target_joint_position);
}

template <int X>
void SoloX<X>::send_target_joint_velocity(
    const Eigen::Ref<Vectord<X>> target_joint_velocity)
{
    robot_->joints->SetDesiredVelocities(target_joint_velocity);
}

template <int X>
void SoloX<X>::send_target_joint_position_gains(
    const Eigen::Ref<Vectord<X>> target_joint_position_gains)
{
    robot_->joints->SetPositionGains(target_joint_position_gains);
}

template <int X>
void SoloX<X>::send_target_joint_velocity_gains(
    const Eigen::Ref<Vectord<X>> target_joint_velocity_gains)
{
    robot_->joints->SetVelocityGains(target_joint_velocity_gains);
}

template <int X>
void SoloX<X>::wait_until_ready()
{
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    static long int count_wait_until_ready = 0;
    while (state_ != SoloState::ready)
    {
        if (count_wait_until_ready % 200 == 0)
        {
            log_->info("Solo::wait_until_ready Getting ready");
        }
        spinner.spin();
        count_wait_until_ready++;
    }
}

template <int X>
bool SoloX<X>::is_ready()
{
    return state_ == SoloState::ready;
}

template <int X>
bool SoloX<X>::request_calibration(const Vectord<X>& home_offset_rad)
{
    log_->info("Solo::request_calibration called");
    Eigen::VectorXd hor = home_offset_rad;
    calib_ctrl_->UpdatePositionOffsets(hor);
    calibrate_request_ = true;
    return true;
}

}  // namespace solo
