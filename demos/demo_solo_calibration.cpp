/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo class in a small demo.
 */


#include "blmc_robots/solo.hpp"
#include "blmc_robots/common_demo_header.hpp"


using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Solo& robot = *(static_cast<Solo*>(robot_void_ptr));

    Eigen::Vector2d joint_index_to_zero;
    joint_index_to_zero[0] = 0.0;
    joint_index_to_zero[1] = 0.0;
    joint_index_to_zero[2] = 0.0;
    joint_index_to_zero[3] = 0.0;
    robot.calibrate(joint_index_to_zero);

    while(!StopDemos)
    {
      
    }

    StopDemos = true;

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int, char**)
{
    enable_ctrl_c();

    real_time_tools::RealTimeThread thread;

    Solo robot;

    robot.initialize();

    rt_printf("controller is set up \n");

    thread.create_realtime_thread(&control_loop, &robot);

    rt_printf("control loop started \n");

    while (true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    return 0;
}
