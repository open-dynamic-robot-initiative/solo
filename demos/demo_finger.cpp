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
#include <iostream>

using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    RealFinger& robot = *(static_cast<RealFinger*>(robot_void_ptr));

    double kp = 0.2;
    double kd = 0.0025;

    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    size_t count = 0;
    while(true)
    {
        // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
        Eigen::Vector3d desired_angles  =
                (robot.get_slider_positions().array() - 0.5) * 2 * M_PI;

        // we implement here a small pd control at the current level
        Eigen::Vector3d desired_torque = kp * (desired_angles - robot.get_angles()) -
                kd * robot.get_angular_velocities();

        // Send the current to the motor
        robot.set_torques(desired_torque);
        robot.send_torques();

        spinner.spin();

        // print -------------------------------------------------------------------
        if ((count % 1000) == 0)
        {
            std::cout << "desired_torque: " << desired_torque.transpose() << std::endl;
            std::cout << "angles: " << robot.get_angles().transpose() << std::endl;
        }
        ++count;
    }//endwhile
}// end control_loop

int main(int argc, char **argv)
{
    RealFinger finger("can0", "can1");
    rt_printf("done creating finger \n");

    real_time_tools::block_memory();
    real_time_tools::RealTimeThread thread;
    real_time_tools::create_realtime_thread(thread, &control_loop, &finger);

    rt_printf("control loop started \n");

    while(true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    return 0;
}
