/**
 * \file common_header.hpp
 * \brief The hardware wrapper of the quadruped
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the TestBench8Motors class which defines the test
 * bench with 8 motors.
 */

#ifndef COMMON_HEADER_HPP
#define COMMON_HEADER_HPP

#include <Eigen/Eigen>
#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>

namespace blmc_robots
{
/**
 * @brief Vector8d shortcut for the eigen vector of size 2.
 */
typedef Eigen::Matrix<double, 2, 1> Vector2d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 8.
 */
typedef Eigen::Matrix<double, 8, 1> Vector8d;

/**
 * @brief CanBus_ptr shortcut for the shared pointer CanBus type
 */
typedef std::shared_ptr<blmc_drivers::CanBus> CanBus_ptr;

/**
 * @brief CanBusMotorBoard_Ptr shortcut for the shared pointer CanBus type
 */
typedef std::shared_ptr<blmc_drivers::CanBusMotorBoard> CanBusMotorBoard_ptr;

/**
 * @brief MotorInterface_ptr shortcut for the shared pointer MotorInterface type
 */
typedef std::shared_ptr<blmc_drivers::MotorInterface> MotorInterface_ptr;

/**
 * @brief Motor_ptr shortcut for the shared pointer Motor type
 */
typedef std::shared_ptr<blmc_drivers::Motor> Motor_ptr;

/**
 * @brief SafeMotor_ptr shortcut for the shared pointer SafeMotor type
 */
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;

/**
 * @brief Slider_ptr shortcut for the linear potentiometer analogue sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

/**
 * @brief Slider_ptr shortcut for the contact sensor. It is also an analogue
 * sensors
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> ContactSensor_ptr;

/**
 * @brief Slider_ptr shortcut for the height sensor. It is also an analogue
 * sensors
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> HeightSensor_ptr;

/**
 * @brief mi, this typdef is used to get the measurements from the blmc api
 */
typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

} // namespace blmc_robots

#endif // COMMON_HEADER_HPP
