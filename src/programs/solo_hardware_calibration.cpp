/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo class in a small demo.
 */

#include "blmc_robots/solo.hpp"
#include "blmc_robots/common_header.hpp"

using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Solo& robot = *(static_cast<Solo*>(robot_void_ptr));

    Eigen::Vector8d joint_index_to_zero;
    joint_index_to_zero.fill(0.0);
    robot.calibrate(joint_index_to_zero);

    long int count = 0;
    while(!StopControl)
    {
      if(count % 200)
      {
        print_vector("Joint Positions", robot.get_joint_positions());
      }
    }

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
