#include <cmath>
#include "real_time_tools/spinner.hpp"
#include "blmc_robots/common_programs_header.hpp"
#include "blmc_robots/solo12.hpp"

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

void Solo12::initialize(const std::string &network_id)
{
  network_id_ = network_id;
  
  // Create the different mapping
  map_joint_id_to_motor_board_id_ = {0, 1, 1, 0, 2, 2, 3, 4, 4, 3, 5, 5};
  map_joint_id_to_motor_port_id_ = {0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

  // Initialize the communication with the main board.
  main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id_);
  main_board_ptr_->Init();

  // create the SpiBus (blmc_drivers wrapper around the MasterBoardInterface)
  spi_bus_ = std::make_shared<blmc_drivers::SpiBus>(main_board_ptr_,
                                                    motor_boards_.size());

  // create the motor board objects:
  for(size_t mb_id = 0 ; mb_id < motor_boards_.size() ; ++mb_id)
  {
      motor_boards_[mb_id] =
          std::make_shared<blmc_drivers::SpiMotorBoard>(spi_bus_, mb_id);
  }

  // Create the motors object. j_id is the joint_id
  for(unsigned j_id = 0; j_id < motors_.size(); ++j_id)
  {
      motors_[j_id] = std::make_shared<blmc_drivers::Motor> (
          motor_boards_[map_joint_id_to_motor_board_id_[j_id]],
          map_joint_id_to_motor_port_id_[j_id]);
  }

  // These are the sliders plugged on the first motor board.
  sliders_[0] = std::make_shared<blmc_drivers::AnalogSensor>(motor_boards_[0], 0);
  sliders_[1] = std::make_shared<blmc_drivers::AnalogSensor>(motor_boards_[0], 1);
  // These data ar the copy of the first ones.
  sliders_[2] = std::make_shared<blmc_drivers::AnalogSensor>(motor_boards_[0], 0);
  sliders_[3] = std::make_shared<blmc_drivers::AnalogSensor>(motor_boards_[0], 1);

  // Create the joint module objects
  joints_.set_motor_array(motors_, motor_torque_constants_, joint_gear_ratios_,
                          joint_zero_positions_, motor_max_current_);

  // Set the maximum joint torque available
  max_joint_torques_ = max_joint_torque_security_margin_ *
                       joints_.get_max_torques().array();

  // fix the polarity to be the same as the urdf model.
  reverse_polarities_ = {true, true, true, false, false, false,
                         true, true, true, false, false, false};
  joints_.set_joint_polarities(reverse_polarities_);

  // The the control gains in order to perform the calibration
  blmc_robots::Vector12d kp, kd;
  kp.fill(3.0);
  kd.fill(0.1);
  joints_.set_position_control_gains(kp, kd);

  // Wait until all the motors are ready.
  spi_bus_->wait_until_ready();
  
  rt_printf("All motors and boards are ready.\n");
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
  // acquire the target joint torques
  joint_target_torques_ = joints_.get_sent_torques();

  // The index angle is not transmitted.
  joint_encoder_index_ = joints_.get_measured_index_angles();

  /**
    * Additional data
    */
  // acquire the slider positions
  for (unsigned i = 0; i < slider_positions_.size(); ++i)
  {
    // acquire the slider
    slider_positions_(i) = sliders_[i]->get_measurement()->newest_element();
  }

  /**
   * The different status.
   */

  // motor board status
  for(size_t i = 0; i < motor_boards_.size(); ++i)
  {
      const blmc_drivers::MotorBoardStatus& motor_board_status =
          motor_boards_[i]->get_status()->newest_element();
      motor_board_enabled_[i] = motor_board_status.system_enabled;
      motor_board_errors_[i] = motor_board_status.error_code;
  }
  // motors status
  for(size_t j_id = 0; j_id < motors_.size(); ++j_id)
  {
      const blmc_drivers::MotorBoardStatus& motor_board_status =
          motor_boards_[map_joint_id_to_motor_board_id_[j_id]]
          ->get_status()->newest_element();
      
      motor_enabled_[j_id] = (map_joint_id_to_motor_port_id_[j_id] == 1) ?
                             motor_board_status.motor2_enabled:
                             motor_board_status.motor1_enabled;
      motor_ready_[j_id] = (map_joint_id_to_motor_port_id_[j_id] == 1) ?
                           motor_board_status.motor2_ready:
                           motor_board_status.motor1_ready;
  }
}

void Solo12::set_max_joint_torques(const double& max_joint_torques)
{
  max_joint_torques_.fill(max_joint_torques);
}

void Solo12::send_target_joint_torque(
    const Eigen::Ref<Vector12d> target_joint_torque)
{
  Vector12d ctrl_torque = target_joint_torque;
  ctrl_torque = ctrl_torque.array().min(max_joint_torques_);
  ctrl_torque = ctrl_torque.array().max(- max_joint_torques_);
  joints_.set_torques(ctrl_torque);
  joints_.send_torques();
}

bool Solo12::calibrate(const Vector12d& home_offset_rad)
{
  // Maximum distance is twice the angle between joint indexes
  double search_distance_limit_rad = 10.0 * (2.0 * M_PI / joint_gear_ratios_(0));
  Vector12d profile_step_size_rad = Vector12d::Constant(0.001);
  HomingReturnCode homing_return_code =
    joints_.execute_homing(search_distance_limit_rad, home_offset_rad,
                           profile_step_size_rad);
  if(homing_return_code == HomingReturnCode::FAILED)
  {
      return false;
  }
  Vector12d zero_pose = Vector12d::Zero();
  joints_.go_to(zero_pose);
  return true;
}

} // namespace blmc_robots
