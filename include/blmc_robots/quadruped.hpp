#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <blmc_robots/common_header.hpp>
#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/slider.hpp>

namespace blmc_robots{

class Quadruped
{
public:

  /**
   * @brief Quadruped is the constructor of the class.
   */
  Quadruped();

  /**
   * @brief initialize the robot by setting aligning the motors and calibrate the
   * sensors to 0
   */
  void initialize();

  /**
   * @brief send_target_torques sends the target currents to the motors
   */
  void send_target_motor_current(
      const Eigen::Ref<Vector8d> target_motor_current);

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
   * @brief set_hardstop2zero_offsets
   *
   */
  void set_hardstop2zero_offsets(
      const Eigen::Ref<Vector8d> hardstop2zero_offsets);

  /**
   * @brief set_start2hardstop_offsets
   *
   */
  void set_start2hardstop_offsets(
      const Eigen::Ref<Vector8d> start2hardstop_offsets);

  /**
   * @brief get_motor_positions
   * @return the current motors positions (rad).
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_motor_positions()
  {
    return motor_positions_;
  }

  /**
   * @brief get_motor_velocities
   * @return the current motors velocities (rad/s).
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_motor_velocities()
  {
    return motor_velocities_;
  }

  /**
   * @brief get_motor_currents
   * @return the current motors currents in (Ampere: A).
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_motor_currents()
  {
    return motor_currents_;
  }

  /**
   * @brief get_motor_target_currents
   * @return the target current motors currents in (Ampere: A).
   */
  const Eigen::Ref<Vector8d> get_motor_target_currents()
  {
    return motor_target_currents_;
  }

  /**
   * @brief get_motor_torques
   * @return the motor torques in Nm
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_motor_torques()
  {
    return motor_torques_;
  }

  /**
   * @brief get_target_motor_torques
   * @return the target motor torques in Nm
   */
  const Eigen::Ref<Vector8d> get_target_motor_torques()
  {
    return motor_target_torques_;
  }

  /**
   * @brief get_motor_inertias
   * @return the motor inertias
   */
  const Eigen::Ref<Vector8d> get_motor_inertias()
  {
    return motor_inertias_;
  }

  /**
   * @brief get_motor_encoder_indexes
   * @return the position of the index of the encoders a the motor level
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_motor_encoder_indexes()
  {
    return motor_encoder_indexes_;
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
   * @brief get_joint_positions
   * @return  the joint angle of each module
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_joint_positions()
  {
    return joint_positions_;
  }

  /**
   * @brief get_joint_velocities
   * @return the joint velocities
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_joint_velocities()
  {
    return joint_velocities_;
  }

  /**
   * @brief get_joint_torques
   * @return the joint torques
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_joint_torques()
  {
    return joint_torques_;
  }

  /**
   * @brief get_joint_torques
   * @return the target joint torques
   */
  const Eigen::Ref<Vector8d> get_joint_target_torques()
  {
    return joint_target_torques_;
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
   * @brief get_hardstop2zero_offsets
   * @return the position where the robot should be in "zero" configuration
   */
  const Eigen::Ref<Vector8d> get_hardstop2zero_offsets()
  {
    return joint_hardstop2zero_offsets_;
  }
  /**
   * @brief get_start2hardstop_offsets_
   * @return the position where the robot should be in "zero" configuration
   */
  const Eigen::Ref<Vector8d> get_start2hardstop_offsets_()
  {
    return joint_start2hardstop_offsets_;
  }
  /**
   * @brief get_slider_positions
   * @return the current sliders positions.
   * prior to any getter to have up to date data.   
   */
  const Eigen::Ref<Eigen::Vector4d> get_slider_positions()
  {
    return slider_positions_;
  }

  /**
   * @brief get_contact_sensors_states
   * @return the state of the contacts states
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector4d> get_contact_sensors_states()
  {
    return contact_sensors_states_;
  }

  /**
   * @brief get_max_current
   * @return the max current that has been hardcoded in the constructor of this
   * class. TODO: parametrize this via yaml or something else.
   */
  const Eigen::Ref<Vector8d> get_max_current()
  {
    return motor_max_current_;
  }

  /**
   * @brief get_max_torque
   * @return the max torque that has been hardcoded in the constructor of this
   * class. TODO: parametrize this via yaml or something else.
   */
  const Eigen::Ref<Vector8d> get_max_joint_torque()
  {
    return joint_max_torque_;
  }
  
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
   * @return This gives the status (enabled/disabled of the onboard control cards)
   */
  const std::array<bool, 4>& get_motor_board_enabled()
  {
    return motor_board_enabled_;
  }

  /**
   * @brief get_motor_board_errors
   * @return This gives the status (enabled/disabled of the onboard control cards)
   */
  const std::array<int, 4>& get_motor_board_errors()
  {
    return motor_board_errors_;
  }

  /**
   * @brief set_max_current sets the maximum current that the motor can
   * to apply.
   * @param max_current contain 1 value per each motors.
   */
  void set_max_current(const Eigen::Ref<Vector8d> max_current)
  {
    motor_max_current_ = max_current;
    for(unsigned i=0 ; i<motors_.size() ; ++i)
    {
      blmc_drivers::SafeMotor* a_safe_motor =
          (blmc_drivers::SafeMotor*)(&motors_[i]);
      a_safe_motor->set_max_current(motor_max_current_(i));
    }
    joint_max_torque_ = motor_max_current_.array() *
                        motor_torque_constants_.array() *
                        joint_gear_ratios_.array();
  }

private:

  /**
    * Motor data
    */

  /**
   * @brief motor_positions_
   */
  Vector8d motor_positions_;
  /**
   * @brief motor_velocities_
   */
  Vector8d motor_velocities_;
  /**
   * @brief motor_currents_
   */
  Vector8d motor_currents_;
  /**
   * @brief motor_torques_
   */
  Vector8d motor_torques_;
  /**
   * @brief motor_inertias_
   */
  Vector8d motor_inertias_;
  /**
   * @brief motor_encoder_indexes_
   */
  Vector8d motor_encoder_indexes_;
  /**
   * @brief motor_target_currents_
   */
  Vector8d motor_target_currents_;
  /**
   * @brief motor_target_torques_
   */
  Vector8d motor_target_torques_;
  /**
   * @brief motor_torque_constants_ are the motor torque constants
   */
  Vector8d motor_torque_constants_;
  /**
   * @brief target_motor_current_tmp_ is used to convert the joint torque to
   * motor current
   */
  Vector8d target_motor_current_tmp_;

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
   * @brief This gives the status (enabled/disabled of the onboard control cards)
   */
  std::array<bool, 4> motor_board_enabled_;

  /**
   * @brief This gives the status (enabled/disabled of the onboard control cards)
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
   * @brief joint_gear_ratios are the joint gear ratios
   */
  Vector8d joint_gear_ratios_;
   /**
   * @brief joint_hardstop2zero_offsets_ are the offsets from the hardstops to 
   * the 0 position (legs straight down). This a constant hardware parameter.
   */
  Vector8d joint_hardstop2zero_offsets_;
  /**
   * @brief joint_start2hardstop_offsets_ are the offsets from the encoders when
   * powered on, till the hardstops. These need to be calibrated each time.
   */
  Vector8d joint_start2hardstop_offsets_;
  /**
   * @brief joint_max_torque_
   */
  Vector8d joint_max_torque_;

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
   * @brief max_current_ is the current limit to be send to the motors,
   * this a safe guard for development
   */
  Vector8d motor_max_current_;

  /**
   * @brief This map for every motor the card number
   */
  std::array<int, 8> motor_to_card_index_;

  /**
   * @brief This map for every motor the card port
   */
  std::array<int, 8> motor_to_card_port_index_;

  /**
    * Drivers communication objects
    */

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
   * receive data
   */
  std::array<MotorInterface_ptr, 8> motors_;

  /**
   * @brief Address the rotation direction of the motor
   */
  std::array<double, 8> polarity_;

  /**
   * @brief sliders_ these are analogue input from linear potentiometers.
   */
  std::array<Slider_ptr, 4> sliders_;
  
  /**
   * @brief contact_sensors_ is the contact sensors at each foot tips. They also
   * are analogue inputs.
   */
  std::array<ContactSensor_ptr, 4> contact_sensors_;
};

} // namespace blmc_robots

#endif // QUADRUPED_H
