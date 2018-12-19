/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the DisentanglementPlatform
 * \author Manuel Wuthrich
 * \date 2018
 *
 * This file declares the DisentanglementPlatform class which defines the test
 * bench with 8 motors.
 */

#pragma once


#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>

#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/slider.hpp>


namespace blmc_robots
{

/**
 * @brief The DisentanglementPlatform class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class DisentanglementPlatform: public BlmcJointModules<2>
{
public:
    enum JointIndexing {base, tip, joint_count};

    DisentanglementPlatform(const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 2>& motors,
           const std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 2>& sliders):
        BlmcJointModules<2>(motors,
                       0.02 * Eigen::Vector2d::Ones(),
                       9.0 * Eigen::Vector2d::Ones(),
                       Eigen::Vector2d::Zero()),
        sliders_(sliders,
                 Eigen::Vector2d::Zero(),
                 Eigen::Vector2d::Ones()) {}


    const Eigen::Vector2d get_slider_positions()
    {
        return sliders_.get_positions();
    }

private:

    Sliders<2> sliders_;
};

} // namespace blmc_robots
