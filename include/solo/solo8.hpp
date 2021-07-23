/**
 * @file
 * @author Julian Viereck
 * @date 2020
 * @brief Solo8 robot with micro drivers.
 * @copyright Copyright (c) 2020, New York University & Max Planck Gesellschaft.
 */

#pragma once

#include <slider_box/serial_reader.hpp>
#include <solo/common_header.hpp>
#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

namespace solo
{
enum Solo8State
{
    initial,
    ready,
    calibrate
};

class Solo8
{
public:
    /**
     * @brief Solo8 is the constructor of the class.
     */
    Solo8();

    /**
     * @brief initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0
     */
    void initialize(const std::string& network_id);

    /**
     * @brief send_target_torques sends the target currents to the motors
     */
    void send_target_joint_torque(
        const Eigen::Ref<Vector8d> target_joint_torque);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    void acquire_sensors();

    /**
     * @brief Asynchrounous calibrate the joints by moving to the next joint index position.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true
     * @return false
     */
    bool request_calibration(const Vector8d& home_offset_rad);

    /**
     * Joint properties
     */

    /**
     * @brief get_motor_inertias
     * @return the motor inertias
     */
    const Eigen::Ref<Vector8d> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants
     * @return the torque constants of each motor
     */
    const Eigen::Ref<Vector8d> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return  the joint gear ratios
     */
    const Eigen::Ref<Vector8d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_max_torque
     * @return the max torque that has been hardcoded in the constructor of this
     * class. TODO: parametrize this via yaml or something else.
     */
    const Eigen::Ref<Vector8d> get_motor_max_current()
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
    const Eigen::Ref<Vector8d> get_joint_positions()
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
    const Eigen::Ref<Vector8d> get_joint_velocities()
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
    const Eigen::Ref<Vector8d> get_joint_torques()
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
    const Eigen::Ref<Vector8d> get_joint_target_torques()
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
    const Eigen::Ref<Vector8d> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

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

    /**
     * Hardware Status
     */

    /**
     * @brief get_motor_enabled
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, 8>& get_motor_enabled()
    {
        return motor_enabled_;
    }

    /**
     * @brief get_motor_ready
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, 8>& get_motor_ready()
    {
        return motor_ready_;
    }

    /**
     * @brief get_motor_board_enabled
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<bool, 4>& get_motor_board_enabled()
    {
        return motor_board_enabled_;
    }

    /**
     * @brief get_motor_board_errors
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<int, 4>& get_motor_board_errors()
    {
        return motor_board_errors_;
    }

private:
    /**
     * Motor data
     */

    /**
     * Joint properties
     */
    Vector8d motor_inertias_;         /**< motors inertia. */
    Vector8d motor_torque_constants_; /**< DCM motor torque constants. */
    Vector8d joint_gear_ratios_;      /**< joint gear ratios (9). */
    Vector8d motor_max_current_;    /**< Max appliable current before the robot
                                       shutdown. */
    Vector8d joint_zero_positions_; /**< Offset to the theoretical "0" pose. */
    Eigen::Array<double, 8, 1>
        max_joint_torques_; /**< Max joint torques (Nm) */
    static const double
        max_joint_torque_security_margin_; /**<  Security margin on the
                                              saturation of the control. */

    /**
     * Hardware status
     */
    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 8> motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 8> motor_ready_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    std::array<bool, 4> motor_board_enabled_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    std::array<int, 4> motor_board_errors_;

    /**
     * Joint data
     */

    /**
     * @brief joint_positions_
     */
    Vector8d joint_positions_;
    /**
     * @brief joint_velocities_
     */
    Vector8d joint_velocities_;
    /**
     * @brief joint_torques_
     */
    Vector8d joint_torques_;
    /**
     * @brief joint_target_torques_
     */
    Vector8d joint_target_torques_;
    /**
     * @brief joint_encoder_index_
     */
    Vector8d joint_encoder_index_;

    /**
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

    /**
     * Drivers communication objects
     */

    /**
     * @brief Reader for serial port to read arduino slider values.
     */
    std::shared_ptr<slider_box::SerialReader> serial_reader_;

    /**
     * @brief Main board drivers.
     *
     * PC <- Ethernet/Wifi -> main board <- SPI -> Motor Board
     */
    std::shared_ptr<MasterBoardInterface> main_board_ptr_;

    /**
     * @brief The odri robot abstraction.
     */
    std::shared_ptr<odri_control_interface::Robot> robot_;

    /**
     * @brief Collection of Joints for solo12.
     */
    std::shared_ptr<odri_control_interface::JointModules> joints_;

    /** @brief Controller to run the calibration procedure */
    std::shared_ptr<odri_control_interface::JointCalibrator> calib_ctrl_;

    /** @brief Indicator if calibration should start. */
    bool calibrate_request_;

    /**
     * @brief For reading the raw slider values from the serial port.
     */
    std::vector<int> slider_positions_vector_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_accelerometer_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_gyroscope_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_attitude_;

    /** @brief base accelerometer. */
    Eigen::Vector3d imu_linear_acceleration_;

    /** @brief base attitude quaternion. */
    Eigen::Vector4d imu_attitude_quaternion_;

    /**
     * @brief Robot Imu drivers.
     */
    std::shared_ptr<odri_control_interface::IMU> imu_;

    /** @brief If the physical estop is pressed or not. */
    bool active_estop_;

    /** @brief If the joint calibration is active or not. */
    bool _is_calibrating;

    /** @brief State of the solo robot. */
    Solo8State state_;
};

}  // namespace solo
