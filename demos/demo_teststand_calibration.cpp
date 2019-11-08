/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Teststand class in a small demo.
 */


#include "blmc_robots/teststand.hpp"
#include "blmc_robots/common_header.hpp"


using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Teststand& robot = *(static_cast<Teststand*>(robot_void_ptr));


    Eigen::Vector2d joint_index_to_zero;
    joint_index_to_zero[0] = -0.354043;
    joint_index_to_zero[1] = -0.243433;
    robot.calibrate(joint_index_to_zero);

    long int count = 0;

    while(!StopDemos)
    {
      if(count % 200)
      {
        print_vector("Joint Positions", robot.get_joint_positions());
      }
    }

    StopDemos = true;

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int, char**)
{
    enable_ctrl_c();

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
