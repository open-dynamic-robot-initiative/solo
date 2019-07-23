#include <math.h>
#include "blmc_robots/teststand.hpp"

namespace blmc_robots{

Teststand::Teststand()
{
  /**
    * Motor data
    */
  motor_inertias_.setZero();
  motor_torque_constants_.setZero();

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
  joint_gear_ratios_.setZero();
  joint_zero_positions_.setZero();

  /**
    * Additional data
    */
  contact_sensors_states_.setZero();
  slider_positions_.setZero();
  motor_max_current_.setZero();

  /**
    * Setup some known data
    */

  // Max current in Amp
  motor_max_current_.fill(12);
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
}


void Teststand::initialize()
{
  // @todo Create a modular way of adding cards
  // initialize the communication with the can cards
  can_buses_[0] = std::make_shared<blmc_drivers::CanBus>("can0");
  can_motor_boards_[0] = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_[0]);

  can_buses_[1] = std::make_shared<blmc_drivers::CanBus>("can1");
  can_motor_boards_[1] = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_[1]);

  // can 0
  contact_sensors_[0] =
      std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[0], 0);
  height_sensors_[0] =
      std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[0], 1);
  // MOTOR_HFE
  motors_[0] = std::make_shared<blmc_drivers::Motor> (can_motor_boards_[0], 1);
  // MOTOR_KFE
  motors_[1] = std::make_shared<blmc_drivers::Motor> (can_motor_boards_[0], 0);
  // JOINT_HFE
  joints_[0] = std::make_shared<BlmcJointModule> (
    motors_[0], motor_torque_constants_[0], joint_gear_ratios_[0], 0.0, false,
    motor_max_current_[0]);
  // JOINT_KFE
  joints_[1] = std::make_shared<BlmcJointModule> (
    motors_[1], motor_torque_constants_[1], joint_gear_ratios_[1], 0.0, false,
    motor_max_current_[1]);

  // can 1
  sliders_[0] =
      std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[1], 0);
  sliders_[1] =
      std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[1], 1);

  // ATI sensor initialization.
  ati_sensor_.initialize();

  // Wait to make sure there is a first package when acquire_sensors() later.
  real_time_tools::Timer::sleep_sec(0.5);

  // Calibrate the zeros of the ati sensor given the current measurement.
  ati_sensor_.setBias();

  // wait until all board are ready and connected
  // can 2
  can_motor_boards_[0]->wait_until_ready();
  // can 3
  can_motor_boards_[1]->wait_until_ready();
}

bool Teststand::acquire_sensors()
{
  try{
    /**
      * Joint data
      */
    for (unsigned i=0 ; i<joints_.size() ; ++i)
    {
      // acquire the joint position
      joint_positions_(i) = joints_[i]->get_measured_angle();
      // acquire the joint velocities
      joint_velocities_(i) = joints_[i]->get_measured_velocity();
      // acquire the joint torques
      joint_torques_(i) = joints_[i]->get_measured_torque();
      // acquire the joint index
      joint_encoder_index_(i) = joints_[i]->get_measured_index_angle();
      // acquire the target joint torques
      joint_target_torques_(i) = joints_[i]->get_sent_torque();
    }

    /**
      * Additional data
      */
    // acquire the slider positions
    for (unsigned i=0 ; i < slider_positions_.size() ; ++i)
    {
      // acquire the slider
      slider_positions_(i) = sliders_[i]->get_measurement()->newest_element();
    }

    for (unsigned i=0 ; i < contact_sensors_states_.size() ; ++i)
    {
      // acquire the current contact states
      contact_sensors_states_(i) =
          contact_sensors_[i]->get_measurement()->newest_element();
      // acquire the height sensor.
      // Transforms the measurement into a rough height measurement of the hip
      // mounting point above the table.
      height_sensors_states_(i) = 
          1.0701053378814493 - 1.0275690598232334 *
          height_sensors_[i]->get_measurement()->newest_element();
    }

    /**
     * Ati sensor readings.
     */
    ati_sensor_.getFT(&ati_force_(0), &ati_torque_(0));

    // Rotate the force and torque values, such that pressing on the force
    // sensor creates a positive force.
    ati_force_(0) *= -1;
    ati_force_(2) *= -1;
    ati_torque_(0) *= -1;
    ati_torque_(2) *= -1;
  }catch(std::exception ex)
  {
    rt_printf("HARDWARE: Something went wrong during the sensor reading.\n");
    rt_printf("error is: %s\n", ex.what());
    return false;
  }
  return true;
}

bool Teststand::send_target_joint_torque(
    const Eigen::Ref<Vector2d> target_joint_torque)
{
  for (unsigned i=0 ; i<joints_.size() ; ++i)
  {
    joints_[i]->set_torque(target_joint_torque(i));
    joints_[i]->send_torque();
  }
  return true;
}

} // namespace blmc_robots
