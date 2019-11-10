/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Teststand class in a small demo.
 */


#include "blmc_robots/teststand.hpp"
#include "common_demo_header.hpp"

using namespace blmc_robots;
typedef ThreadCalibrationData<Teststand> ThreadCalibrationData_t;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr)
{
    ThreadCalibrationData_t* thread_data_ptr =
      (static_cast<ThreadCalibrationData_t*>(thread_data_void_ptr));
    blmc_robots::Vector2d joint_index_to_zero = 
      thread_data_ptr->joint_index_to_zero;
    thread_data_ptr->robot.calibrate(joint_index_to_zero);

    CTRL_C_DETECTED = true;
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop


int main(int, char**)
{
    enable_ctrl_c();

    Teststand robot;
    robot.initialize();

    ThreadCalibrationData_t thread_data(robot);

    rt_printf("controller is set up \n");
    rt_printf("Press enter to launch the calibration \n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    real_time_tools::RealTimeThread thread;
    thread.create_realtime_thread(&control_loop, &thread_data);

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly \n");

    return 0;
}
