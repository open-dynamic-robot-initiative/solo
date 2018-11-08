/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the Finger
 * \author Manuel Wuthrich
 * \date 2018
 *
 * This file declares the Finger class which defines the test
 * bench with 8 motors.
 */

#pragma once


#include <Eigen/Eigen>
#include <blmc_robots/common_header.hh>

namespace blmc_robots
{

/**
 * @brief The Finger class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class Finger
{
public:

  /**
   * @brief mi this typdef is used to get the measurments form the blmc api
   */
  typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

  /**
   * @brief Finger is the constructor of the class.
   */
  Finger();

  /**
   * @brief initialize the robot by setting alining the motors and calibrate the
   * sensors to 0
   */
  void initialize();

  /**
   * @brief send_target_torques sends the target currents to the motors
   */
  void send_target_current(
      const Eigen::Vector3d target_currents);

  /**
   * @brief acquire_sensors acquire all available sensors, WARNING !!!!
   * this method has to be called prior to any gettter to have up to date data.
   */
  void acquire_sensors();

  /**
   * @brief get_motor_positions
   * @return the current motors positions (rad).
   */
  const Eigen::Vector3d get_motor_positions()
  {
    return motor_positions_;
  }

  /**
   * @brief get_motor_velocities
   * @return the current motors velocities (rad/s).
   */
  const Eigen::Vector3d get_motor_velocities()
  {
    return motor_velocities_;
  }

  /**
   * @brief get_motor_currents
   * @return the current motors currents in (Amper: A).
   */
  const Eigen::Vector3d get_motor_currents()
  {
    return motor_currents_;
  }

  /**
   * @brief get_motor_encoder_indexes
   * @return TODO: ask Manuel what is this exactly.
   */
  const Eigen::Vector3d get_motor_encoder_indexes()
  {
    return motor_encoder_indexes_;
  }

  /**
   * @brief get_slider_positions
   * @return the current sliders positions.
   */
  const Eigen::Vector3d get_slider_positions()
  {
    return slider_positions_;
  }

  /**
   * @brief get_max_current
   * @return the maximum current to apply to the motors
   */
  double get_max_current()
  {
    return max_current_;
  }

  /**
   * @brief get_max_current
   * @return the maximum current to apply to the motors
   */
  double get_max_range()
  {
    return max_range_;
  }

private:


  Eigen::Vector3d motor_positions_;
  Eigen::Vector3d motor_velocities_;
  Eigen::Vector3d motor_currents_;
  Eigen::Vector3d motor_encoder_indexes_;
  Eigen::Vector3d slider_positions_;
  Eigen::Vector3d target_currents;


  std::shared_ptr<blmc_drivers::CanBus> can_bus0_;
  std::shared_ptr<blmc_drivers::CanBus> can_bus1_;


  std::shared_ptr<blmc_drivers::CanBusMotorBoard> board0_;
  std::shared_ptr<blmc_drivers::CanBusMotorBoard> board1_;

  double max_current_ ;
  double max_range_ ;

  // we have 4 board with each possessing 2 motors and 2 sliders
  SafeMotor_ptr board0_motor0_;
  SafeMotor_ptr board0_motor1_;
  Slider_ptr board0_slider0_;
  Slider_ptr board0_slider1_;

  SafeMotor_ptr board1_motor0_;
  Slider_ptr board1_slider0_;

};

} // namespace blmc_robots
