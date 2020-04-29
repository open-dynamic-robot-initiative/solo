#include <math.h>
#include "blmc_robots/teststand.hpp"

namespace blmc_robots{

const double Teststand::max_joint_torque_security_margin_ = 0.99;

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
  max_joint_torques_.setZero();

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

  // Create the joint module objects
  joints_.set_motor_array(motors_, motor_torque_constants_, joint_gear_ratios_,
                          joint_zero_positions_, motor_max_current_);

  // set the max torque we saturate to
  max_joint_torques_ = max_joint_torque_security_margin_ *
                       joints_.get_max_torques().array();
  
  // The the control gains in order to perform the calibration
  Eigen::Vector2d kp, kd;
  kp.fill(2.0);
  kd.fill(0.05);
  joints_.set_position_control_gains(kp, kd);

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
  Vector2d ctrl_torque = target_joint_torque;
  ctrl_torque = ctrl_torque.array().min(max_joint_torques_);
  ctrl_torque = ctrl_torque.array().max(- max_joint_torques_);
  joints_.set_torques(ctrl_torque);
  joints_.send_torques();
  return true;
}

bool Teststand::calibrate(const Vector2d& home_offset_rad)
{
  // Maximum distance is twice the angle between joint indexes
  double search_distance_limit_rad = 2.0 * (2.0 * M_PI / 9.0);
  Vector2d profile_step_size_rad = Vector2d::Constant(0.001);
  joints_.execute_homing(search_distance_limit_rad, home_offset_rad,
                         profile_step_size_rad);
  Vector2d zero_pose = Vector2d::Zero();
  joints_.go_to(zero_pose);
  return true; 
}

} // namespace blmc_robots
