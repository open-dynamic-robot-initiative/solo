/**
 * \file solo12.hpp
 * \author Julian Viereck
 * \date 21 November 2019
 * \copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 */

#pragma once

#include <blmc_drivers/devices/spi_motor_board.hpp>
#include <blmc_drivers/serial_reader.hpp>
#include <blmc_drivers/blmc_joint_module.hpp>
#include "blmc_robots/common_header.hpp"
#include <odri_control_interface/robot.hpp>
#include <odri_control_interface/calibration.hpp>

namespace blmc_robots
{
enum Solo12State
{
    initial,
    ready,
    calibrate
};

/**
 * @brief Definition and drivers for the Solo12 robot.
 *
 * Mapping between the DOF and driver boards + motor ports:
 * FL_HAA - motor board 0, motor port 0, motor index 0
 * FL_HFE - motor board 1, motor port 1, motor index 3
 * FL_KFE - motor board 1, motor port 0, motor index 2
 * FR_HAA - motor board 0, motor port 1, motor index 1
 * FR_HFE - motor board 2, motor port 1, motor index 5
 * FR_KFE - motor board 2, motor port 0, motor index 4
 * HL_HAA - motor board 3, motor port 0, motor index 6
 * HL_HFE - motor board 4, motor port 1, motor index 9
 * HL_KFE - motor board 4, motor port 0, motor index 8
 * HR_HAA - motor board 3, motor port 1, motor index 7
 * HR_HFE - motor board 5, motor port 1, motor index 11
 * HR_KFE - motor board 5, motor port 0, motor index 10
 */
class Solo12
{
public:
    /**
     * @brief Solo is the constructor of the class.
     */
    Solo12();

    /**
     * @brief Initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0.
     * @param if_name Interface for connection to hardware.
     */
    void initialize(const std::string& network_id,
                    const std::string& serial_port);

    /**
     * @brief Sets the maximum joint torques.
     */
    void set_max_current(const double& max_current);

    /**
     * @brief send_target_torques sends the target currents to the motors.
     */
    void send_target_joint_torque(
        const Eigen::Ref<Vector12d> target_joint_torque);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    void acquire_sensors();

    /**
     * @brief Calibrate the joints by moving to the next joint index position.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true in case of success.
     * @return false in case of failure.
     */
    bool calibrate(const Vector12d& home_offset_rad);

    /**
     * Joint properties
     */

    /**
     * @brief get_motor_inertias in [kg m^2]
     * @return Motor inertias.
     */
    const Eigen::Ref<Vector12d> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants in []
     * @return Torque constants of each motor.
     */
    const Eigen::Ref<Vector12d> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return Joint gear ratios
     */
    const Eigen::Ref<Vector12d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_max_torque
     * @return the max torque that has been hardcoded in the constructor of this
     * class. @todo Parametrize the maximum current via yaml or something else.
     */
    const Eigen::Ref<Vector12d> get_motor_max_current()
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
    const Eigen::Ref<Vector12d> get_joint_positions()
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
    const Eigen::Ref<Vector12d> get_joint_velocities()
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
    const Eigen::Ref<Vector12d> get_joint_torques()
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
    const Eigen::Ref<Vector12d> get_joint_target_torques()
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
    const Eigen::Ref<Vector12d> get_joint_encoder_index()
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

    /** @brief base attitude from imu.
     * @return
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector3d> get_imu_attitude()
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

    /** @brief base attitude quaternion (ordered {x, y, z, w}) from imu.
     * @return
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Eigen::Vector4d> get_imu_attitude_quaternion()
    {
        return imu_attitude_quaternion_;
    }

    /*
     * Hardware Status
     */

    /**
     * @brief get_motor_enabled
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, 12>& get_motor_enabled()
    {
        return motor_enabled_;
    }

    /**
     * @brief get_motor_ready
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, 12>& get_motor_ready()
    {
        return motor_ready_;
    }

    /**
     * @brief get_motor_board_enabled
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<bool, 6>& get_motor_board_enabled()
    {
        return motor_board_enabled_;
    }

    /**
     * @brief get_motor_board_errors
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<int, 6>& get_motor_board_errors()
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

    /**
     * @brief is_calibrating()
     * @return Returns true if the calibration procedure is running right now.
     */
    bool is_calibrating()
    {
        return _is_calibrating;
    }

private:
    /**
     * Joint properties
     */

    /** @brief Motors inertia. */
    Vector12d motor_inertias_;
    /** @brief DCM motor torque constants. */
    Vector12d motor_torque_constants_;
    /** @brief joint gear ratios (9). */
    Vector12d joint_gear_ratios_;
    /** @brief Max appliable current before the robot shutdown. */
    Vector12d motor_max_current_;
    /** @brief Offset to the theoretical "0" pose. */
    Vector12d joint_zero_positions_;
    /** @brief Max joint torques (Nm) */
    Eigen::Array<double, 12, 1> max_joint_torques_;
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
    std::array<bool, 12> motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 12> motor_ready_;

    /**
     * @brief This gives the status
     * (enabled/disabled of the onboard control cards).
     */
    std::array<bool, 6> motor_board_enabled_;

    /**
     * @brief This gives the status
     * (enabled/disabled of the onboard control cards).
     */
    std::array<int, 6> motor_board_errors_;

    /**
     * Joint data
     */

    /**
     * @brief joint_positions_
     */
    Vector12d joint_positions_;
    /**
     * @brief joint_velocities_
     */
    Vector12d joint_velocities_;
    /**
     * @brief joint_torques_
     */
    Vector12d joint_torques_;
    /**
     * @brief joint_target_torques_
     */
    Vector12d joint_target_torques_;
    /**
     * @brief joint_encoder_index_
     */
    Vector12d joint_encoder_index_;

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
     * @brief For reading the raw slider values from the serial port.
     */
    std::vector<int> slider_positions_vector_;

    /**
     * @brief contact_sensors_ is contact sensors at each feet of teh quadruped.
     */
    Eigen::Vector4d contact_sensors_states_;

    /** @brief This is the name of the network: Left column in ifconfig output
     */
    std::string network_id_;

    /** @brief Map the joint id to the motor board id, @see Solo12 description.
     */
    std::array<int, 12> map_joint_id_to_motor_board_id_;
    /** @brief Map the joint id to the motor port id, @see Solo12 description.
     */
    std::array<int, 12> map_joint_id_to_motor_port_id_;

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

    /** @brief State of the solo robot. */
    Solo12State state_;

    /** @brief Controller to run the calibration procedure */
    std::shared_ptr<odri_control_interface::JointCalibrator > calib_ctrl_;

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
    std::shared_ptr<blmc_drivers::SerialReader> serial_reader_;

    /**
     * @brief The odri robot abstraction.
     */
    std::shared_ptr<odri_control_interface::Robot > robot_;

    /**
     * @brief Collection of Joints for solo12.
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
};

}  // namespace blmc_robots
