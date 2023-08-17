/**
 * \file solo12_hardware_calibration.cpp
 * \brief ...
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo12 class in a small demo.
 */

#include <mutex>
#include "solo/common_programs_header.hpp"
#include "solo/solo12.hpp"

using namespace solo;

struct SharedContent{
    Solo12 robot;
    std::mutex sc_mutex;
    bool print_home_offset;
};

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    SharedContent& shared_content = *(static_cast<SharedContent*>(robot_void_ptr));
    Solo12& robot = shared_content.robot;
    std::mutex& sc_mutex = shared_content.sc_mutex;
    bool& print_home_offset = shared_content.print_home_offset;
    
    // Prints the home-offset angle.
    Vector12d twelve_zeros = Vector12d::Zero();
    long int count = 0;
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    while (!CTRL_C_DETECTED)
    {
        sc_mutex.lock();
        robot.acquire_sensors();
        if (count % 500 == 0 && print_home_offset)
        {
            print_vector("Home offset angle [Rad]", -robot.get_joint_positions());
        }
        ++count;
        robot.send_target_joint_torque(twelve_zeros);
        sc_mutex.unlock();
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

    // Initialize the shared content.
    SharedContent shared_content;
    shared_content.robot.initialize(argv[1]);
    shared_content.sc_mutex.unlock();
    shared_content.print_home_offset = false;

    // Start control thread
    real_time_tools::RealTimeThread thread;
    thread.create_realtime_thread(&control_loop, &shared_content);
    rt_printf("Controller is set up.\n");

    rt_printf("Wait until the robot is ready.\n");
    bool robot_ready = false;
    while (!robot_ready)
    {
        shared_content.sc_mutex.lock();
        robot_ready = shared_content.robot.is_ready();
        shared_content.sc_mutex.unlock();
        real_time_tools::Timer::sleep_sec(0.1);
    }

    rt_printf("Robot is ready, press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    // Calibrate the robot.
    Vector12d twelve_zeros = Vector12d::Zero();
    shared_content.sc_mutex.lock();
    shared_content.robot.request_calibration(twelve_zeros);
    shared_content.sc_mutex.unlock();

    // print the home offset:
    do
    {
        // first sleep before checking the robot state, as it needs some time to
        // get updated after calling request_calibration()
        real_time_tools::Timer::sleep_sec(0.1);
        shared_content.sc_mutex.lock();
        robot_ready = shared_content.robot.is_ready();
        shared_content.sc_mutex.unlock();
    }
    while (!robot_ready);

    shared_content.sc_mutex.lock();
    shared_content.print_home_offset = true;
    shared_content.sc_mutex.unlock();

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly.\n");

    return 0;
}
