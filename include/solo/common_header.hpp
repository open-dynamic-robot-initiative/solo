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

}  // namespace solo
