/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the real Finger robot.
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>

#include <blmc_robots/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/finger_types.hpp>

namespace blmc_robots
{
class RealFingerDriver : public NJointBlmcRobotDriver<3, 2>
{
public:
    RealFingerDriver(const std::string &can_0, const std::string &can_1)
        : RealFingerDriver(create_motor_boards({can_0, can_1}))
    {
    }

private:
    RealFingerDriver(const MotorBoards &motor_boards)
        : NJointBlmcRobotDriver<3, 2>(motor_boards,
                                      create_motors(motor_boards),
                                      {
                                          // MotorParameters
                                          .max_current_A = 2.0,
                                          .torque_constant_NmpA = 0.02,
                                          .gear_ratio = 9.0,
                                      },
                                      {
                                          // CalibrationParameters
                                          .torque_ratio = 0.6,
                                          .control_gain_kp = 3.0,
                                          .control_gain_kd = 0.03,
                                          .position_tolerance_rad = 0.05,
                                          .move_timeout = 2000,
                                      },
                                      Vector(0.08, 0.08, 0.04),
                                      true)
    {
        home_offset_rad_ << -0.54, -0.17, 0.0;
        initial_position_rad_ << 1.5, 1.5, 3.0;
    }

    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }
};

robot_interfaces::FingerTypes::BackendPtr create_real_finger_backend(
    const std::string &can_0,
    const std::string &can_1,
    robot_interfaces::FingerTypes::DataPtr robot_data)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    std::shared_ptr<robot_interfaces::RobotDriver<
        robot_interfaces::FingerTypes::Action,
        robot_interfaces::FingerTypes::Observation>>
        robot = std::make_shared<RealFingerDriver>(can_0, can_1);

    auto backend = std::make_shared<robot_interfaces::FingerTypes::Backend>(
        robot, robot_data, MAX_ACTION_DURATION_S, MAX_INTER_ACTION_DURATION_S);
    backend->set_max_action_repetitions(-1);

    return backend;
}

}  // namespace blmc_robots
