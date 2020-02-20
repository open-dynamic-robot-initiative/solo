#include <cmath>
#include "blmc_robots/solo8.hpp"
#include "blmc_robots/common_programs_header.hpp"

namespace blmc_robots{

const double Solo8::max_joint_torque_security_margin_ = 0.99;

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
  reverse_polarities_.fill(false);
  /**
   * Hardware status
   */
  for(unsigned i=0 ; i<motor_enabled_.size(); ++i)
  {
    motor_enabled_[i] = false;
    motor_ready_[i] = false;
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

void Solo8::initialize(const std::string &network_id)
{
  // Create the different mapping
  map_joint_id_to_motor_board_id_ = {0, 0, 1, 1, 2, 2, 3, 3};
  map_joint_id_to_motor_port_id_ = {0, 1, 1, 0, 1, 0, 0, 1};

  // Initialize the communication with the main board.
  main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id);
  main_board_ptr_->Init();

  std::array<bool, 8> reverse_polarities = {true, true, false, false, true, true, false, false};

  joints_ = std::make_shared<blmc_robots::SpiJointModules<8> >(
    main_board_ptr_,
    map_joint_id_to_motor_board_id_,
    map_joint_id_to_motor_port_id_,
    motor_torque_constants_,
    joint_gear_ratios_,
    joint_zero_positions_,
    motor_max_current_,
    reverse_polarities
  );

  joints_->enable();

  while (!joints_->is_ready() && !CTRL_C_DETECTED)
  {
    joints_->send_torques();
    joints_->acquire_sensors();
    real_time_tools::Timer::sleep_sec(0.001);
  }

  if (joints_->is_ready()) {
    rt_printf("All motors and boards are ready.\n");
  }
}

void Solo8::acquire_sensors()
{
  static int counter = 0;

  joints_->acquire_sensors();

  if (counter++ % 1000 == 0 && main_board_ptr_->IsTimeout()) {
    printf("=== master-board is in timeout.\n");
  }

  /**
    * Joint data
    */
  // acquire the joint position
  joint_positions_ = joints_->get_measured_angles();
  // acquire the joint velocities
  joint_velocities_ = joints_->get_measured_velocities();
  // acquire the joint torques
  joint_torques_ = joints_->get_measured_torques();
  // acquire the joint index
  joint_encoder_index_ = joints_->get_measured_index_angles();
  // acquire the target joint torques
  joint_target_torques_ = joints_->get_sent_torques();

  /**
    * Additional data
    */
  // acquire the slider positions
  // TODO: Acquire via Arduino.

  /**
   * The different status.
   */

  // motor board status
  for(size_t i = 0; i < 4; ++i)
  {
    motor_board_enabled_[i] = main_board_ptr_->motor_drivers[i].get_is_enabled();
    motor_board_errors_[i] = main_board_ptr_->motor_drivers[i].get_error_code();
  }

  // motors status
  motor_enabled_ = joints_->get_motor_enabled();
  motor_ready_ = joints_->get_motor_ready();
}

void Solo8::send_target_joint_torque(
    const Eigen::Ref<Vector8d> target_joint_torque)
{
  joints_->set_torques(target_joint_torque);
  joints_->send_torques();
}

bool Solo8::calibrate(const Vector8d& home_offset_rad)
{
  /** @TODO: Implement calibration procedure. */
  return true;
}

} // namespace blmc_robots
