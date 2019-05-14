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
#include "real_time_tools/thread.hpp"


using namespace blmc_robots;
using namespace robot_interfaces;

typedef std::tuple<std::shared_ptr<Finger>,
std::shared_ptr<Sliders<3>>> FingerAndSliders;


struct Hardware
{
    std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2> motor_boards;
    std::shared_ptr<Finger> finger;
    std::shared_ptr<Sliders<3>> sliders;
};


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware& hardware = *(static_cast<Hardware*>(hardware_ptr));

    // position controller -----------------------------------------------------
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    size_t count = 0;
    while(true)
    {
        Eigen::Vector3d desired_torques = hardware.sliders->get_positions();
        hardware.finger->constrain_and_apply_torques(desired_torques);
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
    return THREAD_FUNCTION_RETURN_VALUE;
}


int main(int argc, char **argv)
{
    Hardware hardware;

    // set up motor boards -----------------------------------------------------
    hardware.motor_boards = RealFinger::create_motor_boards("can0", "can1");

    // set up finger -----------------------------------------------------------
    hardware.finger = std::make_shared<RealFinger>(hardware.motor_boards);

    // set up sliders ----------------------------------------------------------
    hardware.sliders =
            std::make_shared<Sliders<3>>(hardware.motor_boards,
                                         -hardware.finger->get_max_torques(),
                                         hardware.finger->get_max_torques());

    // start real-time control loop --------------------------------------------
    real_time_tools::RealTimeThread thread;
    thread.create_realtime_thread(&control_loop, &hardware);
    rt_printf("control loop started \n");
    thread.join();
    return 0;
}
