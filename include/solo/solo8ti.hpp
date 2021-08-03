/**
 * \file quadruped.hpp
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University & Max Planck Gesellschaft.
 */

#pragma once

#include <blmc_drivers/blmc_joint_module.hpp>
#include <solo/common_header_ti.hpp>

// The robot drivers for building the robot wrapper around.
#include <blmc_drivers/devices/analog_sensor.hpp>
#include <blmc_drivers/devices/motor.hpp>

namespace solo
{
class Solo8TI
{
public:
    /**
     * @brief Solo8 is the constructor of the class.
     */
    Solo8TI();

    /**
     * @brief initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0
     */
    void initialize();

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
     * @brief Calibrate the joints by moving to the next joint index position.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true
     * @return false
     */
    bool calibrate(const Vector8d& home_offset_rad);

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
     * @brief This map for every motor the card number.
     */
    std::array<int, 8> motor_to_card_index_;

    /**
     * @brief This map for every motor the card port.
     */
    std::array<int, 8> motor_to_card_port_index_;

    /**
     * @brief can_buses_ are the 4 can buses on the robot.
     */
    std::array<CanBus_ptr, 4> can_buses_;
    /**
     * @brief can_motor_boards_ are the 4 can motor board.
     */
    std::array<CanBusMotorBoard_ptr, 4> can_motor_boards_;

    /**
     * @brief motors_ are the objects allowing us to send motor commands and
     * receive data.
     */
    std::array<MotorInterface_ptr, 8> motors_;

    /**
     * @brief This is the collection of joints that compose the robot.
     */
    blmc_drivers::BlmcJointModules<8> joints_;

    /**
     * @brief Address the rotation direction of the motor.
     */
    std::array<bool, 8> reverse_polarities_;

    /**
     * @brief sliders_ these are analogue input from linear potentiometers.
     */
    std::array<Slider_ptr, 4> sliders_;

    /**
     * @brief contact_sensors_ is the contact sensors at each foot tips. They
     * also are analogue inputs.
     */
    std::array<ContactSensor_ptr, 4> contact_sensors_;
};

}  // namespace solo
