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
#include "real_time_tools/spinner.hpp"



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
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
    MotorBoards;

    Finger(std::string can_0, std::string can_1):
        Finger(create_motor_boards(can_0, can_1))
    {
    }

    Finger(const MotorBoards& motor_boards):
        Finger(create_motors_and_analog_sensors(motor_boards))
    {
        motor_boards_ = motor_boards;

        /// \todo: is this the right place to calibrate?
        calibrate();
        motor_boards_[0]->pause_motors();
        motor_boards_[1]->pause_motors();
    }

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

    const Eigen::Vector3d get_slider_positions()
    {
        return sliders_.get_positions();
    }

private:
    MotorBoards motor_boards_;
    Sliders<3> sliders_;


    MotorBoards
    create_motor_boards(const std::string& can_0, const std::string& can_1)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
        can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
        can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

        // set up motor boards -------------------------------------------------
        MotorBoards
                motor_boards;
        motor_boards[0] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[0]);
        motor_boards[1] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[1]);

        return motor_boards;
    }


    std::tuple<Motors, AnalogSensors>
    create_motors_and_analog_sensors(const MotorBoards& motor_boards)
    {
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

        motor_boards[0]->wait_until_ready();
        motor_boards[1]->wait_until_ready();

        return std::make_tuple(motors, analog_sensors);
    }

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
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);
        std::vector<Eigen::Vector3d> running_velocities(1000);
        int running_index = 0;
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        while(running_index < 3000 || (sum.maxCoeff() / 1000.0 > 0.001))
        {
            Eigen::Vector3d torques = Eigen::Vector3d::Ones() * -10;
            set_torques(torques);
            send_torques();
            Eigen::Vector3d velocities = get_angular_velocities();
            if (running_index >= 1000)
                sum = sum - running_velocities[running_index % 1000];
            running_velocities[running_index % 1000] = velocities;
            sum = sum + velocities;
            running_index++;
            spinner.spin();
        }
        Eigen::Vector3d angle_offsets = get_angles();
        set_zero_angles(angle_offsets);
    }

};

} // namespace blmc_robots
