/**
 * \file solo8_hardware_calibration.cpp
 * \brief ...
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo8 class in a small demo.
 */

#include "solo/common_programs_header.hpp"
#include "solo/solo8ti.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Solo8TI& robot = *(static_cast<Solo8TI*>(robot_void_ptr));

    solo::Vector8d joint_index_to_zero;
    joint_index_to_zero.fill(0.0);
    robot.calibrate(joint_index_to_zero);

    long int count = 0;
    while (!CTRL_C_DETECTED)
    {
        if (count % 200 == 0)
        {
            robot.acquire_sensors();
            print_vector("Joint Positions", robot.get_joint_positions());
        }
        ++count;
        real_time_tools::Timer::sleep_sec(0.001);
    }

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** /*argv*/)
{
    enable_ctrl_c();

    if (argc != 1)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./solo8ti_hardware_calibration`.");
    }

    real_time_tools::RealTimeThread thread;

    Solo8TI robot;

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
