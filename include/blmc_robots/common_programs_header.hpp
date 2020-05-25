/**
 * \file common_header.hpp
 * \brief The hardware wrapper of the quadruped
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the TestBench8Motors class which defines the test
 * bench with 8 motors.
 */

#pragma once

#include "blmc_robots/common_header.hpp"

namespace blmc_robots
{
/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool CTRL_C_DETECTED(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s is the id of the signal
 */
void my_handler(int)
{
    CTRL_C_DETECTED = true;
}

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
void enable_ctrl_c()
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    CTRL_C_DETECTED = false;
}

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 *
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name, Eigen::Ref<Eigen::VectorXd> v)
{
    v_name += ": [";
    rt_printf("%s", v_name.c_str());
    for (int i = 0; i < v.size(); ++i)
    {
        rt_printf("%0.3f, ", v(i));
    }
    rt_printf("]\n");
}

}  // namespace blmc_robots
