/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Teststand class in a small demo.
 */


#include "blmc_robots/teststand.hpp"
#include "common_header.hpp"


using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Teststand& robot = *(static_cast<Teststand*>(robot_void_ptr));

    rt_printf("control loop started \n");

    double kp = 5.0;
    double kd = 0.1;
    double max_range = 0.75 * M_PI;
    Vector2d desired_joint_position;
    Vector2d desired_torque;

    Teststand::VectorSlider sliders;
    Teststand::VectorSlider sliders_filt;

    std::vector<std::deque<double> > sliders_filt_buffer(
        robot.get_slider_positions().size());
    size_t max_filt_dim = 200;
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }
    size_t count = 0;
    bool success_acquiring_sensor = true;
    bool success_sending_torques = true;
    while (!StopControl && success_acquiring_sensor && success_sending_torques)
    {
        // acquire the sensors
        success_acquiring_sensor = robot.acquire_sensors();
        if (!success_acquiring_sensor)
        {
            rt_printf("cannot access sensor\n");
        }

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
        for (unsigned i = 0; i < 1; ++i)
        {
            // desired_pose[i].push_back
            desired_joint_position(2 * i + 0) =
                max_range * (sliders_filt(i) - 0.5);
            desired_joint_position(2 * i + 1) =
                -2 * max_range * (sliders_filt(i) - 0.5);
        }

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();

        // // Send the current to the motor
        success_sending_torques =
            robot.send_target_joint_torque(desired_torque);
        if (!success_sending_torques)
        {
            rt_printf("cannot send control\n");
        }

        // print -----------------------------------------------------------
        real_time_tools::Timer::sleep_sec(0.001);

        if ((count % 1000) == 0)
        {
            print_vector("des_joint_tau  ", desired_torque);
            print_vector("    joint_pos  ", robot.get_joint_positions());
            print_vector("des_joint_pos  ", desired_joint_position);
            print_vector("sliders        ", sliders);
            print_vector("ati_force      ", robot.get_ati_force());
            print_vector("ati_torque     ", robot.get_ati_torque());
            print_vector("contact_sensors", robot.get_contact_sensors_states());
            print_vector("height_sensors ", robot.get_height_sensors());
            rt_printf("\n");
        }
        ++count;
    }  // endwhile
    // send zero torques after the control loop.
    desired_torque.fill(0.0);
    robot.send_target_joint_torque(desired_torque);
    StopControl = true;

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int, char**)
{
    enable_ctrl_c();

    real_time_tools::RealTimeThread thread;

    Teststand robot;

    robot.initialize();

    rt_printf("controller is set up \n");

    thread.create_realtime_thread(&control_loop, &robot);

    // Wait until the application is killed.
    while (!StopControl)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }
    thread.join();

    rt_printf("Exit cleanly \n");

    return 0;
}
