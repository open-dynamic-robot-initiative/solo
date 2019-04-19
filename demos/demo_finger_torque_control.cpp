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

#include "real_time_tools/spinner.hpp"
#include "robot_interfaces/finger.hpp"
#include "blmc_robots/real_finger.hpp"
#include "blmc_robots/slider.hpp"
#include "real_time_tools/realtime_thread_creation.hpp"


using namespace blmc_robots;
using namespace robot_interfaces;

typedef std::tuple<std::shared_ptr<Finger>,
std::shared_ptr<Sliders<3>>> FingerAndSliders;


static THREAD_FUNCTION_RETURN_TYPE control_loop(
        void* finger_and_sliders_void_ptr)
{
    // cast input arguments to the right format --------------------------------
    FingerAndSliders& finger_and_sliders =
            *(static_cast<FingerAndSliders*>(finger_and_sliders_void_ptr));

    auto finger = std::get<0>(finger_and_sliders);
    auto sliders = std::get<1>(finger_and_sliders);

    // position controller -----------------------------------------------------
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    size_t count = 0;
    while(true)
    {
        Eigen::Vector3d desired_torques = sliders->get_positions();
        finger->constrain_and_apply_torques(desired_torques);
        spinner.spin();

//        // print ---------------------------------------------------------------
//        if ((count % 1000) == 0)
//        {
//            std::cout << "desired_torque: "
//                      << desired_torques.transpose() << std::endl;
//            std::cout << "angles: "
//                      << finger->get_measured_angles().transpose() << std::endl;
//            std::cout << "velocities: "
//                      << finger->get_measured_velocities().transpose() << std::endl;
//        }
//        ++count;
    }
}

int main(int argc, char **argv)
{
    // set up motor boards -----------------------------------------------------
    auto motor_boards = RealFinger::create_motor_boards("can0", "can1");

    // set up finger -----------------------------------------------------------
    auto finger = std::make_shared<RealFinger>(RealFinger(motor_boards));

    // set up sliders ----------------------------------------------------------
    auto sliders = std::make_shared<Sliders<3>>(Sliders<3>(
                                                    motor_boards,
                                                    -finger->get_max_torques(),
                                                    finger->get_max_torques()));

    // start real-time control loop --------------------------------------------
    real_time_tools::RealTimeThread thread;
    FingerAndSliders finger_and_sliders = std::make_tuple(finger, sliders);
    real_time_tools::create_realtime_thread(thread,
                                            &control_loop,
                                            &finger_and_sliders);
    rt_printf("control loop started \n");
    real_time_tools::join_thread(thread);
    return 0;
}
