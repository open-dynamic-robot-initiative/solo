/**
 * \file common_header.hpp
 * \brief The hardware wrapper of the quadruped
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the TestBench8Motors class which defines the test
 * bench with 8 motors.
 */

#pragma once

// read some parameters
#include "yaml_utils/yaml_cpp_fwd.hpp"

// For mathematical operation
#include <Eigen/Eigen>

// manage the exit of the program with ctrl+c
#include <signal.h>  // manage the ctrl+c signal
#include <atomic>    // thread safe flag for application shutdown management

// some real_time_tools in order to have a real time control
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"

// The robot drivers for building the robot wrapper around.
#include <blmc_drivers/devices/analog_sensor.hpp>
#include <blmc_drivers/devices/motor.hpp>

namespace solo
{
/**
 * @brief Vector2d shortcut for the eigen vector of size 1.
 */
typedef Eigen::Matrix<double, 1, 1> Vector1d;

/**
 * @brief Vector2d shortcut for the eigen vector of size 2.
 */
typedef Eigen::Matrix<double, 2, 1> Vector2d;

/**
 * @brief Vector2d shortcut for the eigen vector of size 6.
 */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 8.
 */
typedef Eigen::Matrix<double, 8, 1> Vector8d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 12.
 */
typedef Eigen::Matrix<double, 12, 1> Vector12d;

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
 * @brief Slider_ptr shortcut for the linear potentiometer analog sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

/**
 * @brief ContactSensor_ptr shortcut for the contact sensor. It is also an
 * analog sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> ContactSensor_ptr;

/**
 * @brief HeightSensor_ptr shortcut for the height sensor. It is also an analog
 * sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> HeightSensor_ptr;

/**
 * @brief mi, this typedef is used to get the measurements from the blmc api
 */
typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

}  // namespace solo
