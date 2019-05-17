#include <math.h>
#include "blmc_robots/single_leg.hpp"

namespace blmc_robots{

SingleLeg::SingleLeg()
{
  /**
    * Motor data
    */
  motor_positions_.setZero();
  motor_velocities_.setZero();
  motor_currents_.setZero();
  motor_torques_.setZero();
  motor_inertias_.setZero();
  motor_encoder_indexes_.fill(std::nan(""));
  motor_target_currents_.setZero();
  motor_target_torques_.setZero();
  motor_torque_constants_.setZero();
  target_motor_current_tmp_.setZero();

  /**
    * Joint data
    */
  joint_positions_.setZero();
  joint_velocities_.setZero();
  joint_torques_.setZero();
  joint_target_torques_.setZero();
  joint_gear_ratios_.setZero();
  joint_zero_positions_.setZero();

  /**
    * Additional data
    */
  slider_positions_.setZero();
  motor_max_current_.setZero();

  /**
    * Setup some known data
    */

  // for now this value is very small but it is currently for debug mode
  // TODO: this should go into a YAML file
  motor_max_current_.fill(2.);
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
  joint_max_torque_ = motor_max_current_.array() *
                      motor_torque_constants_.array() *
                      joint_gear_ratios_.array();
}


void SingleLeg::initialize()
{
  // initialize the communication with the can cards
  // for(unsigned i=0 ; i< can_buses_.size() ; ++i)
  for(unsigned i=0 ; i< 1; ++i)
  {
    std::ostringstream oss;
    oss << "can" << i;
    can_buses_[i] = std::make_shared<blmc_drivers::CanBus>(oss.str());
    can_motor_boards_[i] =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_[i]);
  }

  sliders_[0] = std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[0], 0);
  sliders_[1] = std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[0], 1);

  // can 0
  // MOTOR
  motors_[0] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[0], 0, motor_max_current_[0]);
  motors_[1] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[0], 1, motor_max_current_[1]);

  // Call the method to sync the max current with the max torques.
  set_max_current(motor_max_current_);

  for(unsigned i=0 ; i< 1; ++i) {
    can_motor_boards_[i]->wait_until_ready();
  }
  
  // Wait to make sure there is a first package when acquire_sensors() later.
  real_time_tools::Timer::sleep_sec(0.1);
}

void SingleLeg::disable_can_recv_timeout()
{
  for(unsigned i=0 ; i< 1; ++i) {
    can_motor_boards_[i]->disable_can_recv_timeout();
  }
}


void SingleLeg::zero_joint_positions()
{
  joint_zero_positions_ = motor_positions_;
}

void SingleLeg::acquire_sensors()
{
  /**
    * Motor data
    */
  for (unsigned i=0 ; i<motors_.size() ; ++i)
  {
    // acquire the encoder indexes
    if (motors_[i]->get_measurement(mi::encoder_index)->length() == 1) {
      motor_encoder_indexes_(i) = motors_[i]->get_measurement(mi::encoder_index)->newest_element();

      // Automatically use the encoder position to ofset the joint zero position.
      joint_zero_positions_(i) = motor_encoder_indexes_(i);
    }

    // acquire the motors positions
    motor_positions_(i) =
        motors_[i]->get_measurement(mi::position)->newest_element()
        - joint_zero_positions_(i) * joint_gear_ratios_(i) ;
    // acquire the motors velocities
    motor_velocities_(i) =
        motors_[i]->get_measurement(mi::velocity)->newest_element();
    // acquire the motors current
    motor_currents_(i) =
        motors_[i]->get_measurement(mi::current)->newest_element();
    // acquire the last sent current sent
    motor_target_currents_(i) =
        motors_[i]->get_sent_current_target()->newest_element();
  }

  // acquire the actual motor torques
  motor_torques_ = motor_currents_.array() * motor_torque_constants_.array();
  // acquire the last sent motor torques
  motor_target_torques_ = motor_target_currents_.array() *
                          motor_torque_constants_.array();

  /**
    * Joint data
    */
  // acquire the joint position
  joint_positions_ = motor_positions_.array() / joint_gear_ratios_.array();
  // acquire the joint velocities
  joint_velocities_ = motor_velocities_.array() / joint_gear_ratios_.array();
  // acquire the joint torques
  joint_torques_ = motor_torques_.array() * joint_gear_ratios_.array();
  // acquire the target joint torques
  joint_target_torques_ = motor_target_torques_.array() *
                          joint_gear_ratios_.array();
  joint_encoder_index_ = motor_encoder_indexes_.array() /
                          joint_gear_ratios_.arra
    .def("get_joint_velocities", &SingleLeg::get_joint_velocities)y();

    .def("get_joint_velocities", &SingleLeg::get_joint_velocities)

    .def("get_joint_velocities", &SingleLeg::get_joint_velocities)
  /**
    .def("get_joint_velocities", &SingleLeg::get_joint_velocities)
    * Additional data
    */
  // acquire the slider positions
  for (unsigned i=0 ; i < slider_positions_.size() ; ++i)
  {
    // acquire the slider
    slider_positions_(i) = sliders_[i]->get_measurement()->newest_element();
  }
}

void SingleLeg::send_target_motor_current(
    const Eigen::Ref<Vector2d> target_motor_current)
{
  // set up the target current
  for(unsigned i=0 ; i<motors_.size() ; ++i)
  {
    motors_[i]->set_current_target(target_motor_current(i));
  }

  // actually send the torques to the robot
  for(unsigned i=0 ; i<motors_.size() ; ++i)
  {
    motors_[i]->send_if_input_changed();
  }
}

void SingleLeg::send_target_joint_torque(
    const Eigen::Ref<Vector2d> target_joint_torque)
{
  target_motor_current_tmp_ = target_joint_torque.array() /
                              joint_gear_ratios_.array() /
                              motor_torque_constants_.array();
  send_target_motor_current(target_motor_current_tmp_);
}

} // namespace blmc_robots
