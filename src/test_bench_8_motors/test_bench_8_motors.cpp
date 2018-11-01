/**
 * \file test_bench_8_motors.cpp
 * \brief The hardware wrapper of the the test bench with 8 blmc motors
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "blmc_robots/test_bench_8_motors/test_bench_8_motors.hh"

namespace blmc_motors
{

TestBench8Motors::TestBench8Motors()
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


void TestBench8Motors::initialize()
{
  // not needed for system other than xenomai
  osi::initialize_realtime_printing();

  // initialize the communication with the can cards
  can_bus0_ = std::make_shared<blmc_drivers::CanBus>("can0");
  can_bus1_ = std::make_shared<blmc_drivers::CanBus>("can1");
  can_bus2_ = std::make_shared<blmc_drivers::CanBus>("can2");
  can_bus3_ = std::make_shared<blmc_drivers::CanBus>("can3");

  // get all informatino about the control cards
  board0_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus0_);
  board1_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1_);
  board2_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2_);
  board3_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus3_);

  // get the drivers for the motors and the sliders
  // two individual motors on individual leg style mounting on the left of the
  // table
  board0_motor0_  = std::make_shared<blmc_drivers::SafeMotor>   (board0_, 0, max_current_);
  board0_motor1_  = std::make_shared<blmc_drivers::SafeMotor>   (board0_, 1, max_current_);
  board0_slider0_ = std::make_shared<blmc_drivers::AnalogSensor>(board0_, 0);
  board0_slider1_ = std::make_shared<blmc_drivers::AnalogSensor>(board0_, 1);

  // two individual motors with a wheel on top
  board1_motor0_  = std::make_shared<blmc_drivers::SafeMotor>   (board1_, 0, max_current_);
  board1_motor1_  = std::make_shared<blmc_drivers::SafeMotor>   (board1_, 1, max_current_);
  board1_slider0_ = std::make_shared<blmc_drivers::AnalogSensor>(board1_, 0);
  board1_slider1_ = std::make_shared<blmc_drivers::AnalogSensor>(board1_, 1);

  // the leg style mounting
  board2_motor0_  = std::make_shared<blmc_drivers::SafeMotor>   (board2_, 0, max_current_);
  board2_motor1_  = std::make_shared<blmc_drivers::SafeMotor>   (board2_, 1, max_current_);
  board2_slider0_ = std::make_shared<blmc_drivers::AnalogSensor>(board2_, 0);
  board2_slider1_ = std::make_shared<blmc_drivers::AnalogSensor>(board2_, 1);

  // the hopper style mounting
  board3_motor0_  = std::make_shared<blmc_drivers::SafeMotor>   (board3_, 0, max_current_);
  board3_motor1_  = std::make_shared<blmc_drivers::SafeMotor>   (board3_, 1, max_current_);
  board3_slider0_ = std::make_shared<blmc_drivers::AnalogSensor>(board3_, 0);
  board3_slider1_ = std::make_shared<blmc_drivers::AnalogSensor>(board3_, 1);

  Timer<>::sleep_ms(10);
}

void TestBench8Motors::acquire_sensors()
{
  // acquire the motors positions
  motor_positions_(0) = board0_motor0_->get_measurement(mi::position)->newest_element();
  motor_positions_(1) = board0_motor1_->get_measurement(mi::position)->newest_element();
  motor_positions_(2) = board1_motor0_->get_measurement(mi::position)->newest_element();
  motor_positions_(3) = board1_motor1_->get_measurement(mi::position)->newest_element();
  motor_positions_(4) = board2_motor0_->get_measurement(mi::position)->newest_element();
  motor_positions_(5) = board2_motor1_->get_measurement(mi::position)->newest_element();
  motor_positions_(6) = board3_motor0_->get_measurement(mi::position)->newest_element();
  motor_positions_(7) = board3_motor1_->get_measurement(mi::position)->newest_element();

  // acquire the motors velocities
  motor_velocities_(0) = board0_motor0_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(1) = board0_motor1_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(2) = board1_motor0_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(3) = board1_motor1_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(4) = board2_motor0_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(5) = board2_motor1_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(6) = board3_motor0_->get_measurement(mi::velocity)->newest_element();
  motor_velocities_(7) = board3_motor1_->get_measurement(mi::velocity)->newest_element();

  // acquire the motors current
  motor_currents_(0) = board0_motor0_->get_measurement(mi::current)->newest_element();
  motor_currents_(1) = board0_motor1_->get_measurement(mi::current)->newest_element();
  motor_currents_(2) = board1_motor0_->get_measurement(mi::current)->newest_element();
  motor_currents_(3) = board1_motor1_->get_measurement(mi::current)->newest_element();
  motor_currents_(4) = board2_motor0_->get_measurement(mi::current)->newest_element();
  motor_currents_(5) = board2_motor1_->get_measurement(mi::current)->newest_element();
  motor_currents_(6) = board3_motor0_->get_measurement(mi::current)->newest_element();
  motor_currents_(7) = board3_motor1_->get_measurement(mi::current)->newest_element();

  // acquire the motors encoder index
//  motor_encoder_indexes_(0) = board0_motor0_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(1) = board0_motor1_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(2) = board1_motor0_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(3) = board1_motor1_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(4) = board2_motor0_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(5) = board2_motor1_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(6) = board3_motor0_->get_measurement(mi::encoder_index)->newest_element();
//  motor_encoder_indexes_(7) = board3_motor1_->get_measurement(mi::encoder_index)->newest_element();

  // acquire the slider positions
  slider_positions_(0) = board0_slider0_->get_measurement()->newest_element();
  slider_positions_(1) = board0_slider1_->get_measurement()->newest_element();
  slider_positions_(2) = board1_slider0_->get_measurement()->newest_element();
  slider_positions_(3) = board1_slider1_->get_measurement()->newest_element();
  slider_positions_(4) = board2_slider0_->get_measurement()->newest_element();
  slider_positions_(5) = board2_slider1_->get_measurement()->newest_element();
  slider_positions_(6) = board3_slider0_->get_measurement()->newest_element();
  slider_positions_(7) = board3_slider1_->get_measurement()->newest_element();
}

void TestBench8Motors::send_target_current(
    const Vector8d target_currents)
{
  board0_motor0_->set_current_target(target_currents(0));
  board0_motor1_->set_current_target(target_currents(1));
  board1_motor0_->set_current_target(target_currents(2));
  board1_motor1_->set_current_target(target_currents(3));
  board2_motor0_->set_current_target(target_currents(4));
  board2_motor1_->set_current_target(target_currents(5));
  board3_motor0_->set_current_target(target_currents(6));
  board3_motor1_->set_current_target(target_currents(7));

  board0_motor0_->send_if_input_changed();
  board0_motor1_->send_if_input_changed();
  board1_motor0_->send_if_input_changed();
  board1_motor1_->send_if_input_changed();
  board2_motor0_->send_if_input_changed();
  board2_motor1_->send_if_input_changed();
  board3_motor0_->send_if_input_changed();
  board3_motor1_->send_if_input_changed();
}

} // namespace blmc_motors
