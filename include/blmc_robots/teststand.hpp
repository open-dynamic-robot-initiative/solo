/**
 * \file teststand.hpp
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck
 Gesellschaft.

 */

#ifndef TESTSTAND_H
#define TESTSTAND_H

#include <AtiFTSensor.h>
#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/common_header.hpp>

namespace blmc_robots
{
/**
 * @brief The class Teststand is used to control the Teststand robot located
 * at MPI-IS Tuebingen. The robot is composed of a single leg on a vertical
 * rail.
 */
class Teststand
{
public:
    /**
     * @brief Data type containing the Sliders data
     */
    typedef Eigen::Matrix<double, 2, 1> VectorSlider;
    /**
     * @brief This represents the contact sensor
     */
    typedef Eigen::Matrix<double, 1, 1> VectorContact;
    /**
     * @brief This type define the forces from the ati-FT sensors
     */
    typedef Eigen::Matrix<double, 3, 1> VectorAtiForce;
    /**
     * @brief This type define the torqes from the ati-FT sensors
     */
    typedef Eigen::Matrix<double, 3, 1> VectorAtiTorque;

    /**
     * @brief Teststand is the constructor of the class.
     */
    Teststand();

    /**
     * @brief initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0
     */
    void initialize();

    /**
     * @brief send_target_torques sends the target currents to the motors
     */
    bool send_target_joint_torque(
        const Eigen::Ref<Vector2d> target_joint_torque);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    bool acquire_sensors();

    /**
     * @brief This function will run a small controller that will move the
     * joints untils the next joint index and reset the joint zero with this
     * knowledge.
     *
     * @return true if success
     * @return false if failure
     */
    bool calibrate(const Vector2d& home_offset_rad);

    /**
     * @brief get_motor_inertias
     * @return the motor inertias
     */
    const Eigen::Ref<Vector2d> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants
     * @return the torque constants of each motor
     */
    const Eigen::Ref<Vector2d> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_positions
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return  the joint angle of each module
     */
    const Eigen::Ref<Vector2d> get_joint_positions()
    {
        return joint_positions_;
    }

    /**
     * @brief get_joint_velocities
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the joint velocities
     */
    const Eigen::Ref<Vector2d> get_joint_velocities()
    {
        return joint_velocities_;
    }

    /**
     * @brief get_joint_torques
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the joint torques
     */
    const Eigen::Ref<Vector2d> get_joint_torques()
    {
        return joint_torques_;
    }

    /**
     * @brief get_joint_torques
     * @return the target joint torques
     */
    const Eigen::Ref<Vector2d> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return  the joint gear ratios
     */
    const Eigen::Ref<Vector2d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_joint_encoder_index
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return The last observed encoder index in joint coordinates.
     */
    const Eigen::Ref<Vector2d> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

    /**
     * @brief get_zero_positions
     * @return the position where the robot should be in "zero" configuration
     */
    const Eigen::Ref<Vector2d> get_zero_positions()
    {
        return joint_zero_positions_;
    }

    /**
     * @brief get_slider_positions
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the current sliders positions.
     */
    const Eigen::Ref<Vector2d> get_slider_positions()
    {
        return slider_positions_;
    }

    /**
     * @brief get_contact_sensors_states
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the state of the contacts states
     */
    const Eigen::Ref<Vector1d> get_contact_sensors_states()
    {
        return contact_sensors_states_;
    }

    /**
     * @brief get_contact_sensors_states
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the state of the contacts states
     */
    const Eigen::Ref<Vector1d> get_height_sensors()
    {
        return height_sensors_states_;
    }

    /**
     * @brief get_max_current
     * @return the max current that has been hardcoded in the constructor of
     * this class. TODO: parametrize this via yaml or something else.
     */
    const Eigen::Ref<Vector2d> get_max_current()
    {
        return motor_max_current_;
    }

    /**
     * @brief Get the ati_force_ object
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return const Eigen::Ref<VectorAtiForce>
     */
    const Eigen::Ref<VectorAtiForce> get_ati_force()
    {
        return ati_force_;
    }

    /**
     * @brief Get the ati_torque_ object
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return const Eigen::Ref<VectorAtiTorque>
     */
    const Eigen::Ref<VectorAtiTorque> get_ati_torque()
    {
        return ati_torque_;
    }

private:
    /**
     * @brief ATI sensor.
     */
    ati_ft_sensor::AtiFTSensor ati_sensor_;

    /**
     * @brief 3D linear force from the ATI FT sensor
     */
    VectorAtiForce ati_force_;

    /**
     * @brief 3D torque measured from the ATI FT sensor
     */
    VectorAtiTorque ati_torque_;

    /**
     * Motor data
     */

    /**
     * @brief motor_inertias_
     */
    Vector2d motor_inertias_;
    /**
     * @brief motor_torque_constants_ are the motor torque constants
     */
    Vector2d motor_torque_constants_;

    /**
     * Joint data
     */

    /**
     * @brief joint_positions_ is the measured data from the onboard card
     * converted at the joint level.
     */
    Vector2d joint_positions_;
    /**
     * @brief joint_velocities_ is the measured data from the onboard card
     * converted at the joint level.
     */
    Vector2d joint_velocities_;
    /**
     * @brief joint_torques_ is the measured data from the onboard card
     * converted at the joint level.
     */
    Vector2d joint_torques_;
    /**
     * @brief joint_target_torques_ is the last given command to be sent.
     */
    Vector2d joint_target_torques_;
    /**
     * @brief joint_gear_ratios are the joint gear ratios
     */
    Vector2d joint_gear_ratios_;
    /**
     * @brief joint_encoder_index_ The last observed encoder_index at the
     * joints.
     */
    Vector2d joint_encoder_index_;

    /**
     * @brief joint_zero_positions_ is the configuration considered as zero
     * position
     */
    Vector2d joint_zero_positions_;

    /**
     * Additional data
     */

    /**
     * @brief slider_positions_ is the position of the linear potentiometer.
     * Can be used as a joystick input.
     */
    Vector2d slider_positions_;

    /**
     * @brief contact_sensors_ is contact sensors at the foot
     */
    Vector1d contact_sensors_states_;

    /**
     * @brief height_sensors_ is the height position of the base.
     */
    Vector1d height_sensors_states_;

    /**
     * @brief max_current_ is the maximum current that can be sent to the
     * motors, this a safe guard for development
     */
    Vector2d motor_max_current_;

    /**
     * @brief max_joint_torques_ (Nm)
     */
    Eigen::Array2d max_joint_torques_;
    /**
     * @brief Security margin on the saturation of the control.
     */
    static const double max_joint_torque_security_margin_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 4> motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 4> motor_ready_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards)
     */
    std::array<bool, 2> motor_board_enabled_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards)
     */
    std::array<int, 2> motor_board_errors_;

    /**
     * Drivers communication objects
     */

    /**
     * @brief can_buses_ are the 2 can buses on the robot.
     */
    std::array<CanBus_ptr, 2> can_buses_;

    /**
     * @brief can_motor_boards_ are the 2 can motor board.
     */
    std::array<CanBusMotorBoard_ptr, 2> can_motor_boards_;

    /**
     * @brief motors_ are the objects allowing us to send motor commands and
     * receive data
     */
    std::array<MotorInterface_ptr, 2> motors_;

    /**
     * @brief joint_ptrs_ are the objects allowing us to send commands and
     * receive data at the joint level. It also ones some self calibration
     * routines.
     */
    BlmcJointModules<2> joints_;

    /**
     * @brief sliders_ these are analogue input from linear potentiometers.
     */
    std::array<Slider_ptr, 2> sliders_;

    /**
     * @brief contact_sensors_ is the contact sensors at each foot tips. They
     * also are analogue inputs.
     */
    std::array<ContactSensor_ptr, 1> contact_sensors_;

    /**
     * @brief contact_sensors_ is the contact sensors at each foot tips. They
     * also are analogue inputs.
     */
    std::array<HeightSensor_ptr, 1> height_sensors_;
};

}  // namespace blmc_robots

#endif  // Teststand_H
