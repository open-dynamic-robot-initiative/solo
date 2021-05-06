/**
 * \file solo12_hardware_calibration.cpp
 * \brief ...
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo12 class in a small demo.
 */

#include "solo/common_programs_header.hpp"
#include "solo/solo12.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Solo12& robot = *(static_cast<Solo12*>(robot_void_ptr));

    // Wait until the robot is ready.
    robot.wait_until_ready();

    // Calibrate the robot.
    Vector12d twelve_zeros = Vector12d::Zero();
    bool good_calibration = robot.calibrate(twelve_zeros);

    // Prints the home-offset angle.
    long int count = 0;
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    while (!CTRL_C_DETECTED && good_calibration)
    {
        robot.acquire_sensors();
        if (count % 200 == 0)
        {
            print_vector("Home offset angle [Rad]", -robot.get_joint_positions());
        }
        robot.send_target_joint_torque(twelve_zeros);
        spinner.spin();
    }

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_solo12_calibration "
            "network_id`.");
    }

    enable_ctrl_c();

    real_time_tools::RealTimeThread thread;

    Solo12 robot;

    robot.initialize(argv[1], "banana");

    rt_printf("Controller is set up.\n");
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    thread.create_realtime_thread(&control_loop, &robot);

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly.\n");

    return 0;
}
