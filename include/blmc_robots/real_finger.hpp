/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the RealFinger
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.

 * This file declares the RealFinger class which defines the test
 * bench with 8 motors.
 */

#pragma once

#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <tuple>

#include <blmc_robots/blmc_joint_module.hpp>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

#include <robot_interfaces/finger.hpp>

namespace blmc_robots
{

class RealFinger : public robot_interfaces::Finger
{
public:
    typedef robot_interfaces::Finger::Vector Vector;

    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
        MotorBoards;

    static std::shared_ptr<Finger> create(const std::string &can_0,
                                          const std::string &can_1)
    {
        auto finger = std::make_shared<RealFinger>(can_0, can_1);
        return finger;
    }

    RealFinger(const std::string &can_0, const std::string &can_1) : RealFinger(create_motor_boards(can_0, can_1))
    {
    }

    RealFinger(const MotorBoards &motor_boards) : RealFinger(create_motors(motor_boards))
    {
        motor_boards_ = motor_boards;
        pause();

        max_torque_ = 2.0 * 0.02 * 9.0;

        calibrate();
        pause();
    }

private:
    RealFinger(const Motors &motors) : joint_modules_(motors,
                                                      0.02 * Vector::Ones(),
                                                      9.0 * Vector::Ones(),
                                                      Vector::Zero()) {}

public:

    virtual Observation get_latest_observation()
    {
        Observation observation;
        observation.angle = get_measured_angles();
        observation.velocity = get_measured_velocities();
        observation.torque = get_measured_torques();
        return observation;
    }


    Vector get_measured_torques() const
    {
        return joint_modules_.get_measured_torques();
    }

    Vector get_measured_angles() const
    {
        return joint_modules_.get_measured_angles();
    }

    Vector get_measured_velocities() const
    {
        return joint_modules_.get_measured_velocities();
    }


    static MotorBoards
    create_motor_boards(const std::string &can_0, const std::string &can_1)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
        can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
        can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

        // set up motor boards -------------------------------------------------
        MotorBoards motor_boards;
        for (size_t i = 0; i < 2; i++)
        {
            motor_boards[i] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[i],
                                                                 1000,
                                                                 10); /// \TODO: reduce the timeout further!!
        }
        for (size_t i = 0; i < 2; i++)
        {
            motor_boards[i]->wait_until_ready();
        }

        return motor_boards;
    }

protected:
    Eigen::Vector3d max_angles_;

    void apply_torques(const Vector &desired_torques)
    {
        /// \todo: the safety checks are now being done in here, but should
        /// come outside
        joint_modules_.set_torques(desired_torques);
        joint_modules_.send_torques();
    }

    virtual void apply_action(const Action &action)
    {
        double start_time_sec = real_time_tools::Timer::get_current_time_sec();
        joint_modules_.set_torques(action);
        joint_modules_.send_torques();
        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);
    }

    static Motors
    create_motors(const MotorBoards &motor_boards)
    {
        // set up motors -------------------------------------------------------
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }

    /// \todo: calibrate needs to be cleaned and should probably not be here
    /// there are two identical copies in disentanglement_platform and finger,
    /// which is disgusting.

    /**
     * @brief this is an initial calibration procedure executed before the actual
     * control loop to have angle readings in an absolute coordinate frame.
     *
     * The finger reaches its joint limits by applying a constant negative torque
     * simultaneously in all the joints. Be aware that this procedure assumes safe
     * motors where the maximum velocities are constrained.
     *
     * The calibration procedure is finished once the joint velocities are zero
     * according to a moving average estimation of them.
     */
    void calibrate()
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. without the velocity and
        /// torque limitation in the motor this would be very unsafe
        Vector angle_offsets;
        {
            int averaging_length = 100;
            std::vector<Vector> running_velocities(averaging_length);
            int running_index = 0;
            Vector sum = Vector::Zero();
            while (running_index < 3000 || (sum.maxCoeff() / 100.0 > 0.001))
            {
                Vector torques = -1 * get_max_torques();
                TimeIndex t = append_desired_action(torques);
                Vector velocities = get_observation(t).velocity;
                if (running_index >= averaging_length)
                    sum = sum - running_velocities[running_index % averaging_length];
                running_velocities[running_index % averaging_length] = velocities;
                sum = sum + velocities;
                running_index++;
            }
            int count = 0;
            int linearly_decrease_time_steps = 1000;
            int zero_torque_time_steps = 500;

            /// \todo: this is yet another hack. we release the tip
            /// joint before the others such that the tip joint and the center joint
            /// dont fight each other
            count = 0;
            Vector torques;
            while (count < linearly_decrease_time_steps)
            {
                torques = -get_max_torques();

                torques[2] = ((linearly_decrease_time_steps -
                               count + 0.0) /
                              linearly_decrease_time_steps) *
                             torques[2];

                TimeIndex t = append_desired_action(torques);
                wait_until_time_index(t);
                count++;
            }
            count = 0;
            while (count < zero_torque_time_steps)
            {
                TimeIndex t = append_desired_action(torques);
                wait_until_time_index(t);
                count++;
            }

            angle_offsets = get_observation(current_time_index()).angle;
            joint_modules_.set_zero_angles(angle_offsets);
        }

        {

            Eigen::Vector3d starting_position;
            starting_position << 1.5, 1.5, 3.0;

            double kp = 0.4;
            double kd = 0.0025;
            int count = 0;
            Eigen::Vector3d last_diff(std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            while (count < 2000)
            {
                Eigen::Vector3d diff = starting_position - get_observation(current_time_index()).angle;

                // we implement here a small pd control at the current level
                Eigen::Vector3d desired_torque = kp * diff -
                                                 kd * get_measured_velocities();

                // Send the current to the motor
                TimeIndex t = append_desired_action(desired_torque);
                wait_until_time_index(t);
                if (count % 100 == 0)
                {
                    Eigen::Vector3d diff_difference = last_diff - diff;
                    if (std::abs(diff_difference.norm()) < 1e-5)
                        break;
                    last_diff = diff;
                }
                count++;
            }
            pause();
        }
    }

private:
    BlmcJointModules<3> joint_modules_;
    MotorBoards motor_boards_;
};

} // namespace blmc_robots
