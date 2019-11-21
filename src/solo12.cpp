#include <cmath>
#include "blmc_robots/solo.hpp"

namespace blmc_robots{

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

  /**
   * Hardware status
   */
  for(unsigned i=0 ; i<motor_enabled_.size(); ++i)
  {
    motor_enabled_[i] = false;
    motor_ready_[i] = false;
    motor_to_card_index_[i] = 0;
    motor_to_card_port_index_[i] = 0;
  }
  for(unsigned i=0 ; i<motor_board_enabled_.size(); ++i)
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
  motor_max_current_.fill(4.0); // TODO: set as paramters?
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
}

void Solo12::initialize(const std::string &if_name)
{
  // Initialize the communication with the main board.
  main_board_ptr = std::make_shared<MasterBoardInterface>(if_name);
  main_board_ptr->Init();

  /**
   * Mapping between the DOF and driver boards + motor ports

   * FL_HAA - driver 0, motor port 0
   * FL_HFE - driver 1, motor port 1
   * FL_KFE - driver 1, motor port 0
   * FR_HAA - driver 0, motor port 1
   * FR_HFE - driver 2, motor port 1
   * FR_KFE - driver 2, motor port 0
   * HL_HAA - driver 3, motor port 0
   * HL_HFE - driver 4, motor port 1
   * HL_KFE - driver 4, motor port 0
   * HR_HAA - driver 3, motor port 1
   * HR_HFE - driver 5, motor port 1
   * HR_KFE - driver 5, motor port 0
   */
  motor_to_card_index_[0] = 0; // FL_HAA
  motor_to_card_index_[1] = 1; // FL_HFE
  motor_to_card_index_[2] = 1; // FL_KFE
  motor_to_card_index_[3] = 0; // FR_HAA
  motor_to_card_index_[4] = 2; // FR_HFE
  motor_to_card_index_[5] = 2; // FR_KFE
  motor_to_card_index_[6] = 3; // HL_HAA
  motor_to_card_index_[7] = 4; // HL_HFE
  motor_to_card_index_[8] = 4; // HL_KFE
  motor_to_card_index_[9] = 3; // HR_HAA
  motor_to_card_index_[10] = 5; // HR_HFE
  motor_to_card_index_[11] = 5; // HR_KFE

  motor_to_card_port_index_[0] = 1; // FL_HAA
  motor_to_card_port_index_[1] = 1; // FL_HFE
  motor_to_card_port_index_[2] = 0; // FL_KFE
  motor_to_card_port_index_[3] = 1; // FR_HAA
  motor_to_card_port_index_[4] = 1; // FR_HFE
  motor_to_card_port_index_[5] = 0; // FR_KFE
  motor_to_card_port_index_[6] = 1; // HL_HAA
  motor_to_card_port_index_[7] = 1; // HL_HFE
  motor_to_card_port_index_[8] = 0; // HL_KFE
  motor_to_card_port_index_[9] = 1; // HL_HAA
  motor_to_card_port_index_[10] = 1; // HR_HFE
  motor_to_card_port_index_[11] = 0; // HR_KFE

  // Create the motors object.
  for(unsigned i=0; i<motors_.size() ; ++i)
  {
    motors_[i] = std::make_shared<blmc_drivers::SafeMotor> (
      can_motor_boards_[motor_to_card_index_[i]],
      motor_to_card_port_index_[i],
      motor_max_current_[6]
    );
  }

  // Create the joint module objects
  joints_.set_motor_array(motors_, motor_torque_constants_, joint_gear_ratios_,
                          joint_zero_positions_, motor_max_current_);

  // Set the maximum joint torque available
  max_joint_torques_ = max_joint_torque_security_margin_ *
                       joints_.get_max_torques().array();

  // fix the polarity to be the same as the urdf model.
  reverse_polarities_[0] = true;  // FL_HFE
  reverse_polarities_[1] = true;  // FL_KFE
  reverse_polarities_[2] = false; // FR_HFE
  reverse_polarities_[3] = false; // FR_KFE
  reverse_polarities_[4] = true;  // HL_HFE
  reverse_polarities_[5] = true;  // HL_KFE
  reverse_polarities_[6] = false; // HR_HFE
  reverse_polarities_[7] = false; // HR_KFE
  joints_.set_joint_polarities(reverse_polarities_);

  // The the control gains in order to perform the calibration
  blmc_robots::Vector8d kp, kd;
  kp.fill(3.0);
  kd.fill(0.05);
  joints_.set_position_control_gains(kp, kd);

  // wait until all board are ready and connected
  for(unsigned i=0 ; i<can_buses_.size() ; ++i)
  {
    can_motor_boards_[i]->wait_until_ready();
  }
}

void Solo12::acquire_sensors()
{
  /**
    * Joint data
    */
  // acquire the joint position
  joint_positions_ = joints_.get_measured_angles();
  // acquire the joint velocities
  joint_velocities_ = joints_.get_measured_velocities();
  // acquire the joint torques
  joint_torques_ = joints_.get_measured_torques();
  // acquire the joint index
  joint_encoder_index_ = joints_.get_measured_index_angles();
  // acquire the target joint torques
  joint_target_torques_ = joints_.get_sent_torques();

  /**
    * Additional data
    */
  // acquire the slider positions
  for (unsigned i=0 ; i<slider_positions_.size() ; ++i)
  {
    // acquire the slider
    slider_positions_(i) = sliders_[i]->get_measurement()->newest_element();
    // acquire the current contact states
    contact_sensors_states_(i) =
        contact_sensors_[i]->get_measurement()->newest_element();
  }

  /**
   * The different status.
   */
  blmc_drivers::MotorBoardStatus FL_status =
      can_motor_boards_[3]->get_status()->newest_element();
  blmc_drivers::MotorBoardStatus FR_status =
      can_motor_boards_[0]->get_status()->newest_element();
  blmc_drivers::MotorBoardStatus HL_status =
      can_motor_boards_[2]->get_status()->newest_element();
  blmc_drivers::MotorBoardStatus HR_status =
      can_motor_boards_[1]->get_status()->newest_element();

  motor_board_enabled_[0] = static_cast<bool>(FL_status.system_enabled); // FL board
  motor_board_enabled_[1] = static_cast<bool>(FR_status.system_enabled); // FR board
  motor_board_enabled_[2] = static_cast<bool>(HL_status.system_enabled); // HL board
  motor_board_enabled_[3] = static_cast<bool>(HR_status.system_enabled); // HR board

  motor_board_errors_[0] = static_cast<int>(FL_status.error_code); // FL board
  motor_board_errors_[1] = static_cast<int>(FR_status.error_code); // FR board
  motor_board_errors_[2] = static_cast<int>(HL_status.error_code); // HL board
  motor_board_errors_[3] = static_cast<int>(HR_status.error_code); // HR board

  motor_enabled_[0] = static_cast<bool>(FL_status.motor2_enabled); // FL_HFE
  motor_enabled_[1] = static_cast<bool>(FL_status.motor1_enabled); // FL_KFE
  motor_enabled_[2] = static_cast<bool>(FR_status.motor2_enabled); // FR_HFE
  motor_enabled_[3] = static_cast<bool>(FR_status.motor1_enabled); // FR_KFE
  motor_enabled_[4] = static_cast<bool>(HL_status.motor2_enabled); // HL_HFE
  motor_enabled_[5] = static_cast<bool>(HL_status.motor1_enabled); // HL_KFE
  motor_enabled_[6] = static_cast<bool>(HR_status.motor2_enabled); // HR_HFE
  motor_enabled_[7] = static_cast<bool>(HR_status.motor1_enabled); // HR_KFE

  motor_ready_[0] = static_cast<bool>(FL_status.motor2_ready); // FL_HFE
  motor_ready_[1] = static_cast<bool>(FL_status.motor1_ready); // FL_KFE
  motor_ready_[2] = static_cast<bool>(FR_status.motor2_ready); // FR_HFE
  motor_ready_[3] = static_cast<bool>(FR_status.motor1_ready); // FR_KFE
  motor_ready_[4] = static_cast<bool>(HL_status.motor2_ready); // HL_HFE
  motor_ready_[5] = static_cast<bool>(HL_status.motor1_ready); // HL_KFE
  motor_ready_[6] = static_cast<bool>(HR_status.motor2_ready); // HR_HFE
  motor_ready_[7] = static_cast<bool>(HR_status.motor1_ready); // HR_KFE
}

void Solo12::send_target_joint_torque(
    const Eigen::Ref<Vector8d> target_joint_torque)
{
  Vector8d ctrl_torque = target_joint_torque;
  ctrl_torque = ctrl_torque.array().min(max_joint_torques_);
  ctrl_torque = ctrl_torque.array().max(- max_joint_torques_);
  joints_.set_torques(ctrl_torque);
  joints_.send_torques();
}

bool Solo12::calibrate(const Vector8d& home_offset_rad)
{
  // Maximum distance is twice the angle between joint indexes
  double search_distance_limit_rad = 2.0 * (2.0 * M_PI / 9.0);
  double profile_step_size_rad=0.001;
  joints_.execute_homing(search_distance_limit_rad, home_offset_rad,
                         profile_step_size_rad);
  Vector8d zero_pose = Vector8d::Zero();
  joints_.go_to(zero_pose);
  return true;
}

} // namespace blmc_robots
