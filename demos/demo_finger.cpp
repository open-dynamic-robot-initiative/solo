/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the RealFinger class in a small demo.
 */

#include "real_time_tools/spinner.hpp"
#include "blmc_robots/real_finger.hpp"
#include <blmc_robots/slider.hpp>
#include "robot_interfaces/finger.hpp"
#include <iostream>

using namespace blmc_robots;
using namespace robot_interfaces;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* finger_void_ptr)
{
    RealFinger& finger = *(static_cast<RealFinger*>(finger_void_ptr));

    double kp = 0.2;
    double kd = 0.0025;

    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    size_t count = 0;
    while(true)
    {
        // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
        Eigen::Vector3d desired_angles  =
                (finger.get_slider_positions().array() - 0.5) * 2 * M_PI;

        // we implement here a small pd control at the current level
        Eigen::Vector3d desired_torque = kp * (desired_angles - finger.get_angles()) -
                kd * finger.get_angular_velocities();

        // Send the current to the motor
        finger.set_torques(desired_torque);
        finger.send_torques();

        spinner.spin();

        // print -------------------------------------------------------------------
        if ((count % 1000) == 0)
        {
            std::cout << "desired_torque: " << desired_torque.transpose() << std::endl;
            std::cout << "angles: " << finger.get_angles().transpose() << std::endl;
        }
        ++count;
    }//endwhile
}// end control_loop

int main(int argc, char **argv)
{
    // set up motor boards -----------------------------------------------------
    auto motor_boards = RealFinger::create_motor_boards("can0", "can1");

    // set up finger -----------------------------------------------------------
    auto finger = std::make_shared<RealFinger>(RealFinger(motor_boards));
    rt_printf("done creating finger \n");

    // set up sliders -----------------------------------------------------------
    auto sliders = std::make_shared<Sliders<3>>(Sliders<3>(motor_boards,
                                                           Eigen::Vector3d::Zero(),
                                                           Eigen::Vector3d::Ones()));


    real_time_tools::block_memory();
    real_time_tools::RealTimeThread thread;
    real_time_tools::create_realtime_thread(thread, &control_loop, finger.get());

    rt_printf("control loop started \n");

    while(true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    return 0;
}
