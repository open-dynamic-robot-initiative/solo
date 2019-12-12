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

void map_sliders(Eigen::Ref<Eigen::Vector4d> sliders, Eigen::Ref<Vector12d> sliders_out)
{
    for (int i = 0; i < 4; i++)
    {
        sliders_out(3 * i + 0) = sliders(0);
        sliders_out(3 * i + 1) = sliders(1);
        sliders_out(3 * i + 2) = 2. * (1. - sliders(1));

        if (i >= 2) {
            sliders_out(3 * i + 1) *= -1;
            sliders_out(3 * i + 2) *= -1;
        }
    }
    // Swap the hip direction.
    sliders_out(3) *= -1;
    sliders_out(9) *= -1;
}

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* args)
{
    Solo12& robot = *static_cast<Solo12*>(args);

    // Using conversion from PD gains from example.cpp
    double kp = 3.0 * 9 * 0.025;
    double kd = 0.1 * 9 * 0.025;
    double max_range = M_PI;
    Vector12d desired_joint_position;
    Vector12d desired_torque_tmp;
    Vector12d desired_torque;

    desired_torque.setZero();

    Vector12d sliders;
    Vector12d sliders_filt;
    Vector12d sliders_zero;
    std::array<bool, 12> motor_enabled;

    std::vector<std::deque<double> > sliders_filt_buffer(12);
    size_t max_filt_dim = 200;
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }

    robot.acquire_sensors();
    map_sliders(robot.get_slider_positions(), sliders_zero);

    rt_printf("control loop started \n");

    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot.acquire_sensors();

        // acquire the motor enabled signal.
        motor_enabled = robot.get_motor_enabled();

        // HACK: Only read sliders from card0 if motors report enabled.
        if (motor_enabled[0] && motor_enabled[1])
        {
            // acquire the slider signal
            map_sliders(robot.get_slider_positions(), sliders);
        }

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
        for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
        {
            // desired_pose[i].push_back
            desired_joint_position(i) =
                max_range * (sliders_filt(i) - sliders_zero(i));
        }

        // we implement here a small pd control at the current level
        desired_torque_tmp =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();

        // // HACK: Due to unstable SPI, only update torque for enabled motors.
        for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
        {
            if (motor_enabled[i])
            {
                desired_torque(i) = desired_torque_tmp(i);
            }
        }

        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            printf("\33[H\33[2J"); //clear screen
            print_vector("sliders_filt", sliders_filt);
            print_vector("sliders_zero", sliders_zero);
            print_vector("sliders_raw ", robot.get_slider_positions());
            print_vector("des_joint_tau", desired_torque);
            print_vector("    joint_pos", robot.get_joint_positions());
            print_vector("    joint_vel", robot.get_joint_velocities());
            print_vector("des_joint_pos", desired_joint_position);
            fflush(stdout);
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
        rt_printf("Usage: demo_solo12 interface_name \n");
        return -1;
    }

    real_time_tools::RealTimeThread thread;
    enable_ctrl_c();

    Solo12 robot;
    robot.initialize( std::string(argv[1]) );
    robot.set_max_joint_torques(0.5);
    
    rt_printf("controller is set up \n");
    thread.create_realtime_thread(&control_loop, &robot);

    rt_printf("control loop started \n");
    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    thread.join();

    return 0;
}
