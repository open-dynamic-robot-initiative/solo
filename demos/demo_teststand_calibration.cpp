/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Teststand class in a small demo.
 */

// Exiting on ctrl+c
#include <signal.h>
#include <atomic>

#include <Eigen/Eigen>
#include <cmath>
#include <deque>
#include <numeric>
#include "blmc_robots/teststand.hpp"
#include "real_time_tools/timer.hpp"

using namespace blmc_robots;

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s
 */
void my_handler(int)
{
    StopDemos = true;
}

void print_vector(std::string v_name, Eigen::Ref<Eigen::VectorXd> v)
{
    v_name += ": [";
    rt_printf("%s", v_name.c_str());
    for (int i = 0; i < v.size(); ++i)
    {
        rt_printf("%f, ", v(i));
    }
    rt_printf("]\n");
}

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Teststand& robot = *(static_cast<Teststand*>(robot_void_ptr));

    Eigen::Vector2d zero_to_index_angle;
    zero_to_index_angle[0] = 0.354043;
    zero_to_index_angle[1] = 0.243433;
    robot.calibrate(zero_to_index_angle);

    StopDemos = true;

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int, char**)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    real_time_tools::RealTimeThread thread;

    Teststand robot;

    robot.initialize();

    rt_printf("controller is set up \n");
    rt_printf("Press enter to launch the calibration \n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    thread.create_realtime_thread(&control_loop, &robot);

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly \n");

    return 0;
}
