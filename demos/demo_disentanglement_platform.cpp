/**
 * \file demo_disentanglement_platform.cpp
 * \brief a demo illustrating how to use the RealDisentanglementPlatform class
 * \author Manuel Wuthrich
 * \date 2019
 *
 * This file uses the RealDisentanglementPlatform class in a small demo.
 */

#include <iostream>
#include <tuple>
#include <Eigen/Core>

#include "real_time_tools/spinner.hpp"
#include "blmc_robots/disentanglement_platform.hpp"
#include "real_time_tools/realtime_thread_creation.hpp"


using namespace blmc_robots;



static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_ptr)
{
    // cast input arguments to the right format --------------------------------
    std::shared_ptr<RealDisentanglementPlatform>& robot =
            *(static_cast<std::shared_ptr<RealDisentanglementPlatform>*>(robot_ptr));

    // position controller -----------------------------------------------------
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    size_t count = 0;
    while(true)
    {
        Eigen::Vector3d desired_torque = 0 * robot->get_max_torques();

        robot->constrain_and_apply_torques(desired_torque);
        spinner.spin();

        // print ---------------------------------------------------------------
        if ((count % 1000) == 0)
        {
            std::cout << "desired_torque: "
                      << desired_torque.transpose() << std::endl;
            std::cout << "angles: "
                      << robot->get_measured_angles().transpose() << std::endl;
        }
        ++count;
    }
}

int main(int argc, char **argv)
{
    // set up motor boards -----------------------------------------------------
    auto motor_boards =
            RealDisentanglementPlatform::create_motor_boards("can0", "can1");

    // set up disentanglement_platform -----------------------------------------------------------
    auto robot = std::make_shared<RealDisentanglementPlatform>(motor_boards);

    // start real-time control loop --------------------------------------------
    real_time_tools::RealTimeThread thread;
    real_time_tools::create_realtime_thread(thread,
                                            &control_loop,
                                            &robot);
    rt_printf("control loop started \n");
    real_time_tools::join_thread(thread);
    return 0;
}
