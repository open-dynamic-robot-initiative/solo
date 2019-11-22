/**
 * \file demo_solo12.cpp
 * \brief Implements basic PD controller reading slider values.
 * \author Julian Viereck
 * \date 21 November 2019
 *
 * This file uses the Solo12 class in a small demo.
 */


#include "blmc_robots/solo12.hpp"
#include "blmc_robots/common_programs_header.hpp"

using namespace blmc_robots;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void*)
{
    Solo12 robot;
    // TODO: Forward commandline argument here.
    robot.initialize("ens4", 4);

    rt_printf("control loop started \n");

    double kp = 5.0;
    double kd = 1.0;
    double max_range = M_PI;
    Vector12d desired_joint_position;
    Vector12d desired_torque;

    desired_torque.setZero();

    Eigen::Vector4d sliders;
    Eigen::Vector4d sliders_filt;
    Eigen::Vector4d sliders_zero;

    std::vector<std::deque<double> > sliders_filt_buffer(
        robot.get_slider_positions().size());
    size_t max_filt_dim = 200;
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }

    robot.acquire_sensors();
    sliders_zero = robot.get_slider_positions();

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
            desired_joint_position(2 * i) =
                max_range * (sliders_filt(i) - sliders_zero(i));
            desired_joint_position(2 * i + 1) = desired_joint_position(2 * i);
        }

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();


        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            print_vector("sliders_filt", sliders);
            print_vector("des_joint_tau", desired_torque);
            print_vector("    joint_pos", robot.get_joint_positions());
            print_vector("des_joint_pos", desired_joint_position);
        }
        ++count;

        // Send the current to the motor
        // desired_torque.setZero();
        robot.send_target_joint_torque(desired_torque);

        real_time_tools::Timer::sleep_sec(0.001);
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc < 2) {
        rt_printf("Usage: demo_solo12 interface_name\n");
        return -1;
    }

    real_time_tools::RealTimeThread thread;
    enable_ctrl_c();

    rt_printf("controller is set up \n");
    thread.create_realtime_thread(&control_loop);

    Eigen::Vector4d sliders;
    Vector12d desired_torque;
    int counter = 0;

    rt_printf("control loop started \n");
    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    return 0;
}
