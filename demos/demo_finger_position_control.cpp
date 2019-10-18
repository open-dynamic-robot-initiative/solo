/**
 * \file demo_finger.cpp
 * \brief a demo illustrating how to use the RealFinger class
 * \author Manuel Wuthrich
 * \date 2019
 *
 * This file uses the RealFinger class in a small demo.
 */

#include <iostream>
#include <tuple>

#include <real_time_tools/spinner.hpp>
#include <robot_interfaces/finger_types.hpp>
#include <blmc_robots/real_finger_driver.hpp>
#include <blmc_robots/slider.hpp>
#include <real_time_tools/thread.hpp>

using namespace blmc_robots;
using namespace robot_interfaces;

typedef std::tuple<std::shared_ptr<FingerTypes::Frontend>,
                   std::shared_ptr<Sliders<3>>>
    FingerAndSliders;

static THREAD_FUNCTION_RETURN_TYPE control_loop(
    void *finger_and_sliders_void_ptr)
{
    // cast input arguments to the right format --------------------------------
    FingerAndSliders &finger_and_sliders =
        *(static_cast<FingerAndSliders *>(finger_and_sliders_void_ptr));

    auto finger = std::get<0>(finger_and_sliders);
    auto sliders = std::get<1>(finger_and_sliders);

    // position controller -----------------------------------------------------
    double kp = 0.2;
    double kd = 0.0025;

    FingerTypes::Action desired_action = FingerTypes::Action::Zero();
    while (true)
    {
        TimeIndex t = finger->append_desired_action(desired_action);
        desired_action.torque = kp * (sliders->get_positions() -
                                      finger->get_observation(t).position) -
                                kd * finger->get_observation(t).velocity;

        // print ---------------------------------------------------------------
        if ((t % 1000) == 0)
        {
            std::cout << "desired_torque: "
                      << finger->get_desired_action(t).torque << std::endl;
            std::cout << "angles: " << finger->get_observation(t).position
                      << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    // // set up motor boards
    // ----------------------------------------------------- auto motor_boards =
    // RealFinger::create_motor_boards("can0", "can1");

    // // set up finger
    // ----------------------------------------------------------- auto finger =
    // RealFinger::create("can0", "can1");

    // // set up sliders
    // ---------------------------------------------------------- auto sliders =
    //     std::make_shared<Sliders<3>>(Sliders<3>(
    //         motor_boards,
    //         -0.5 * M_PI * Eigen::Vector3d::Ones(),
    //         1.5 * M_PI * Eigen::Vector3d::Ones()));

    // // start real-time control loop
    // --------------------------------------------
    // real_time_tools::RealTimeThread thread;
    // FingerAndSliders finger_and_sliders = std::make_tuple(finger, sliders);
    // thread.create_realtime_thread(&control_loop,
    //                               &finger_and_sliders);
    // rt_printf("control loop started \n");
    // thread.join();
    // return 0;
}
