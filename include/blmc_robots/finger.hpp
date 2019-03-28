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
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <tuple>


#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/slider.hpp>


namespace blmc_robots
{

/**
 * @brief The Finger class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class Finger: public BlmcJointModules<3>
{
public:
    enum JointIndexing {base, center, tip, joint_count};

    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3>
    AnalogSensors;

    Finger(std::string can_0, std::string can_1):
        Finger(create_motors_and_analog_sensors(can_0, can_1)){}

    Finger(const std::tuple<Motors, AnalogSensors>& motors_and_analog_sensors):
        Finger(std::get<0>(motors_and_analog_sensors),
               std::get<1>(motors_and_analog_sensors)) {}


    Finger(const Motors& motors, const AnalogSensors& analog_sensors):
        BlmcJointModules<3>(motors,
                            0.02 * Eigen::Vector3d::Ones(),
                            9.0 * Eigen::Vector3d::Ones(),
                            Eigen::Vector3d::Zero()),
        sliders_(analog_sensors,
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Ones()) {}


public:
    const Eigen::Vector3d get_slider_positions()
    {
        return sliders_.get_positions();
    }




private:
    Sliders<3> sliders_;



    std::tuple<Motors, AnalogSensors>
    create_motors_and_analog_sensors(std::string can_0, std::string can_1)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
        can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
        can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

        // set up motor boards -------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
                motor_boards;
        motor_boards[0] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[0]);
        motor_boards[1] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[1]);
        motor_boards[0]->wait_until_ready();
        motor_boards[1]->wait_until_ready();


        // set up motors -------------------------------------------------------
        Motors motors;
        double max_motor_velocity = 20;
        motors[0]  = std::make_shared<blmc_drivers::SafeMotor>(motor_boards[0],
                0, 2.0, 1000, max_motor_velocity);
        motors[1]  = std::make_shared<blmc_drivers::SafeMotor>(motor_boards[0],
                1, 2.0, 1000, max_motor_velocity);
        motors[2]  = std::make_shared<blmc_drivers::SafeMotor>(motor_boards[1],
                0, 2.0, 1000, max_motor_velocity);

        // set up sliders ------------------------------------------------------
        AnalogSensors analog_sensors;
        analog_sensors[0] = std::make_shared<blmc_drivers::AnalogSensor>(
                    motor_boards[0], 0);
        analog_sensors[1] = std::make_shared<blmc_drivers::AnalogSensor>(
                    motor_boards[0], 1);
        analog_sensors[2] = std::make_shared<blmc_drivers::AnalogSensor>(
                    motor_boards[1], 0);

        return std::make_tuple(motors, analog_sensors);
    }

};

} // namespace blmc_robots
