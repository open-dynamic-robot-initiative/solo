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


namespace blmc_robots
{

/**
 * @brief The Finger class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class Finger
{
public:

    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;
    enum MotorIndexing {base, center, tip, joint_count};

    Finger(const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3>& motors,
                   const std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3>& sliders)
    {
        motors_ = motors;
        sliders_ = sliders;

        modules_[MotorIndexing::base]  = std::make_shared<BlmcModule> (motors_[MotorIndexing::base], 0.02, 9.0, 0.0);
        modules_[MotorIndexing::center]  = std::make_shared<BlmcModule> (motors_[MotorIndexing::center], 0.02, 9.0, 0.0);
        modules_[MotorIndexing::tip]  = std::make_shared<BlmcModule> (motors_[MotorIndexing::tip], 0.02, 9.0, 0.0);
    }


    void send_torques(const Eigen::Vector3d& desired_torques)
    {
        for(size_t i = 0; i < joint_count; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
        for(size_t i = 0; i < joint_count; i++)
        {
            modules_[i]->send_torque();
        }
    }

    Eigen::Vector3d get_torques() const
    {
        Eigen::Vector3d torques;

        for(size_t i = 0; i < joint_count; i++)
        {
            torques(i) = modules_[i]->get_torque();
        }
        return torques;
    }

    Eigen::Vector3d get_angles() const
    {
        Eigen::Vector3d positions;

        for(size_t i = 0; i < joint_count; i++)
        {
            positions(i) = modules_[i]->get_angle();
        }
        return positions;
    }

    Eigen::Vector3d get_angular_velocities() const
    {
        Eigen::Vector3d velocities;

        for(size_t i = 0; i < joint_count; i++)
        {
            velocities(i) = modules_[i]->get_angular_velocity();
        }
        return velocities;
    }


    Eigen::Vector3d get_index_angles() const
    {
        Eigen::Vector3d index_angles;

        for(size_t i = 0; i < joint_count; i++)
        {
            index_angles(i) = modules_[i]->get_index_angle();
        }
        return index_angles;
    }

    const Eigen::Vector3d get_slider_positions()
    {
        Eigen::Vector3d slider_positions;

        for(size_t i = 0; i < joint_count; i++)
        {
            slider_positions(i) =
                    sliders_[i]->get_measurement()->newest_element();
        }
        return slider_positions;
    }


private:
    std::array<std::shared_ptr<BlmcModule>, 3> modules_;

    // we have 4 board with each possessing 2 motors and 2 sliders
    std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> motors_;
    std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3> sliders_;
};

} // namespace blmc_robots
