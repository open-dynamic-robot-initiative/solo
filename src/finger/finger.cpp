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
    zero_angle_ = 0;
    gear_ratio_ = 9.0;
    motor_constant_ = 0.02; // Nm/A

    max_current_ = 2.0 ;
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



  modules_[MotorIndexing::base]  = std::make_shared<BlmcModule> (motors_[MotorIndexing::base], 0.02, 9.0, 0.0);
  modules_[MotorIndexing::center]  = std::make_shared<BlmcModule> (motors_[MotorIndexing::center], 0.02, 9.0, 0.0);
  modules_[MotorIndexing::tip]  = std::make_shared<BlmcModule> (motors_[MotorIndexing::tip], 0.02, 9.0, 0.0);




  Timer<>::sleep_ms(10);
}



} // namespace blmc_robots
