/**
 * \file
 * \brief Driver for a "two-joint" robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>

#include <blmc_robots/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/n_joint_robot_types.hpp>

namespace blmc_robots
{
/**
 * @brief Driver for two joints.
 *
 * Driver for a double BLMC joint.  Mostly intended for testing purposes.
 */

class TwoJointDriver : public NJointBlmcRobotDriver<2, 1>
{
public:
    /**
     * @brief Constructor
     *
     * @param can_port  Name of the CAN port to which the joint is connected
     *     (e.g. "can0")
     * @param home_offset_rad  Home offset of the joint.  This is the offset
     *     between the home position (= encoder index found during homing) and
     *     the desired zero position.
     */


    TwoJointDriver(const std::string &can_port,
                   Vector &home_offset_rad)
        : TwoJointDriver(create_motor_boards({can_port}),
                         home_offset_rad)
    {
    }

private:
    TwoJointDriver(const MotorBoards &motor_boards,
                   Vector &home_offset_rad)
        : NJointBlmcRobotDriver<2, 1>(motor_boards,
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
                                          .position_tolerance_rad = 0.05,
                                          .move_timeout = 2000,
                                      },
                                      Vector(0.08, 0.08),
                                      Vector(3.0, 3.0),
                                      Vector(0.03, 0.03),
                                      true)
    {
        home_offset_rad_ << home_offset_rad;
    }

    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);

        return motors;
    }
    
};

robot_interfaces::NJointRobotTypes<2>::BackendPtr create_two_joint_backend(
    const std::string &can_0,
    robot_interfaces::NJointRobotTypes<2>::Vector &home_offset_rad,
    robot_interfaces::NJointRobotTypes<2>::DataPtr robot_data)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    std::shared_ptr<robot_interfaces::RobotDriver<
        robot_interfaces::NJointRobotTypes<2>::Action,
        robot_interfaces::NJointRobotTypes<2>::Observation>>
        robot = std::make_shared<TwoJointDriver>(can_0, home_offset_rad);

    auto backend =
        std::make_shared<robot_interfaces::NJointRobotTypes<2>::Backend>(
            robot,
            robot_data,
            MAX_ACTION_DURATION_S,
            MAX_INTER_ACTION_DURATION_S);
    backend->set_max_action_repetitions(std::numeric_limits<uint32_t>::max());

    return backend;
}

}  // namespace blmc_robots
