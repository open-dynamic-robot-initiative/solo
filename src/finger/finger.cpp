/**
 * \file test_bench_8_motors.cpp
 * \brief The hardware wrapper of the the test bench with 8 blmc motors
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the Finger class.
 */

#include "blmc_robots/finger/finger.hpp"

#include <limits>

namespace blmc_robots
{

Finger::Finger()
{
  max_current_ = 1.0 ;
  max_range_ = 2.0;

  motor_positions_.setZero();
  motor_velocities_.setZero();
  motor_currents_.setZero();
  motor_encoder_indexes_.setZero();
  slider_positions_.setZero();
  target_currents.setZero();

}


void Finger::initialize()
{
  // not needed for system other than xenomai
  osi::initialize_realtime_printing();

  // initialize the communication with the can cards
  auto can_bus_0 = std::make_shared<blmc_drivers::CanBus>("can0");
  auto can_bus_1 = std::make_shared<blmc_drivers::CanBus>("can1");

  // get all informatino about the control cards
  auto board_0 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_0);
  auto board_1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_1);

  // get the drivers for the motors and the sliders
  // two individual motors on individual leg style mounting on the left of the
  // table
  motors_[MotorIndexing::base]  = std::make_shared<blmc_drivers::SafeMotor>   (board_0, 0, max_current_);
  motors_[MotorIndexing::center]  = std::make_shared<blmc_drivers::SafeMotor>   (board_0, 1, max_current_);
  sliders_[MotorIndexing::base] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 0);
  sliders_[MotorIndexing::center] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 1);

  // two individual motors with a wheel on top
  motors_[MotorIndexing::tip]  = std::make_shared<blmc_drivers::SafeMotor>   (board_1, 0, max_current_);
  sliders_[MotorIndexing::tip] = std::make_shared<blmc_drivers::AnalogSensor>(board_1, 0);

  Timer<>::sleep_ms(10);
}

void Finger::acquire_sensors()
{
  // acquire the motors positions
  motor_positions_(0) = motors_[MotorIndexing::base]->get_measurement(mi::position)->newest_element();
  motor_positions_(1) = motors_[MotorIndexing::center]->get_measurement(mi::position)->newest_element();
  motor_positions_(2) = motors_[MotorIndexing::tip]->get_measurement(mi::position)->newest_element();

  // acquire the motors velocities
  motor_velocities_(0) = motors_[MotorIndexing::base]->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(1) = motors_[MotorIndexing::center]->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(2) = motors_[MotorIndexing::tip]->get_measurement(mi::velocity)->newest_element();

  // acquire the motors current
  motor_currents_(0) = motors_[MotorIndexing::base]->get_measurement(mi::current)->newest_element();
  motor_currents_(1) = motors_[MotorIndexing::center]->get_measurement(mi::current)->newest_element();
  motor_currents_(2) = motors_[MotorIndexing::tip]->get_measurement(mi::current)->newest_element();


  // acquire the motors encoder index
  motor_encoder_indexes_(0) =
      motors_[MotorIndexing::base]->get_measurement(mi::encoder_index)->length() > 0 ?
        motors_[MotorIndexing::base]->get_measurement(mi::encoder_index)->newest_element():
        std::numeric_limits<double>::quiet_NaN();

  motor_encoder_indexes_(1) =
      motors_[MotorIndexing::center]->get_measurement(mi::encoder_index)->length() > 0 ?
        motors_[MotorIndexing::center]->get_measurement(mi::encoder_index)->newest_element():
        std::numeric_limits<double>::quiet_NaN();

  motor_encoder_indexes_(2) =
      motors_[MotorIndexing::tip]->get_measurement(mi::encoder_index)->length() > 0 ?
        motors_[MotorIndexing::tip]->get_measurement(mi::encoder_index)->newest_element():
        std::numeric_limits<double>::quiet_NaN();


  // acquire the slider positions
  slider_positions_(0) = sliders_[MotorIndexing::base]->get_measurement()->newest_element();
  slider_positions_(1) = sliders_[MotorIndexing::center]->get_measurement()->newest_element();
  slider_positions_(2) = sliders_[MotorIndexing::tip]->get_measurement()->newest_element();
}

void Finger::send_target_current(
    const Eigen::Vector3d target_currents)
{
  motors_[MotorIndexing::base]->set_current_target(target_currents(0));
  motors_[MotorIndexing::center]->set_current_target(target_currents(1));
  motors_[MotorIndexing::tip]->set_current_target(target_currents(2));


  motors_[MotorIndexing::base]->send_if_input_changed();
  motors_[MotorIndexing::center]->send_if_input_changed();
  motors_[MotorIndexing::tip]->send_if_input_changed();
}

} // namespace blmc_robots
