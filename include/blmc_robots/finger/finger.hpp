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
#include <math.h>

#include <blmc_robots/blmc_module/blmc_module.hpp>
#include <blmc_robots/slider/slider.hpp>


namespace blmc_robots
{

/**
 * @brief The Finger class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class Finger: public BlmcModules<3>
{
public:
    enum JointIndexing {base, center, tip, joint_count};

    Finger(const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3>& motors,
           const std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3>& sliders):
        BlmcModules<3>(motors,
                       0.02 * Eigen::Vector3d::Ones(),
                       9.0 * Eigen::Vector3d::Ones(),
                       Eigen::Vector3d::Zero()),
        sliders_(sliders,
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Ones()) {}


    const Eigen::Vector3d get_slider_positions()
    {
        return sliders_.get_positions();
    }

private:

    Sliders<3> sliders_;
};

} // namespace blmc_robots
