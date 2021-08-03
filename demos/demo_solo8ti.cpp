/**
 * \file demo_Solo8TI.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo8TI class in a small demo.
 */

#include <numeric>
#include "solo/common_programs_header.hpp"
#include "solo/solo8ti.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Solo8TI& robot = *(static_cast<Solo8TI*>(robot_void_ptr));

    double kp = 5.0;
    double kd = 1.0;
    double max_range = M_PI;
    Vector8d desired_joint_position;
    Vector8d desired_torque;

    Eigen::Vector4d sliders;
    Eigen::Vector4d sliders_filt;

    std::vector<std::deque<double> > sliders_filt_buffer(
        robot.get_slider_positions().size());
    size_t max_filt_dim = 200;
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }
    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot.acquire_sensors();

        // aquire the slider signal
        sliders = robot.get_slider_positions();
        // filter it
        for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
        {
            if (sliders_filt_buffer[i].size() >= max_filt_dim)
            {
                sliders_filt_buffer[i].pop_front();
            }
            sliders_filt_buffer[i].push_back(sliders(i));
            sliders_filt(i) = std::accumulate(sliders_filt_buffer[i].begin(),
                                              sliders_filt_buffer[i].end(),
                                              0.0) /
                              (double)sliders_filt_buffer[i].size();
        }

        // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
        for (unsigned i = 0; i < 4; ++i)
        {
            // desired_pose[i].push_back
            desired_joint_position(2 * i) = max_range * (sliders_filt(i) - 0.5);
            desired_joint_position(2 * i + 1) =
                max_range * (sliders_filt(i) - 0.5);
        }

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();

        // Send the current to the motor
        robot.send_target_joint_torque(desired_torque);

        // print -----------------------------------------------------------
        real_time_tools::Timer::sleep_sec(0.001);

        if ((count % 1000) == 0)
        {
            print_vector("des_joint_tau", desired_torque);
            print_vector("    joint_pos", robot.get_joint_positions());
            print_vector("des_joint_pos", desired_joint_position);
        }
        ++count;
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int, char**)
{
    enable_ctrl_c();

    real_time_tools::RealTimeThread thread;

    Solo8TI robot;

    robot.initialize();

    rt_printf("controller is set up \n");

    thread.create_realtime_thread(&control_loop, &robot);

    rt_printf("control loop started \n");

    while (true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    thread.join();

    return 0;
}
