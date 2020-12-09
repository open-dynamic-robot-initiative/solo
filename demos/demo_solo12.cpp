/**
 * \file demo_solo12.cpp
 * \brief Implements basic PD controller reading slider values.
 * \author Julian Viereck
 * \date 21 November 2019
 *
 * This file uses the Solo12 class in a small demo.
 */

#include <numeric>
#include "blmc_robots/common_programs_header.hpp"
#include "blmc_robots/solo12.hpp"
#include "common_demo_header.hpp"

using namespace blmc_robots;
typedef ThreadCalibrationData<Solo12> ThreadCalibrationData_t;

void map_sliders(Eigen::Ref<Eigen::Vector4d> sliders,
                 Eigen::Ref<Vector12d> sliders_out)
{
    double slider_A = sliders(0) - 0.5;
    double slider_B = sliders(1);
    for (int i = 0; i < 4; i++)
    {
        sliders_out(3 * i + 0) = slider_A;
        sliders_out(3 * i + 1) = slider_B;
        sliders_out(3 * i + 2) = 2. * (1. - slider_B);

        if (i >= 2)
        {
            sliders_out(3 * i + 1) *= -1;
            sliders_out(3 * i + 2) *= -1;
        }
    }
    // Swap the hip direction.
    sliders_out(3) *= -1;
    sliders_out(9) *= -1;
}

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr)
{
    ThreadCalibrationData_t* thread_data_ptr =
        (static_cast<ThreadCalibrationData_t*>(thread_data_void_ptr));
    std::shared_ptr<Solo12> robot = thread_data_ptr->robot;

    // Using conversion from PD gains from example.cpp
    double kp = 5.0 * 9 * 0.025;
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
    size_t max_filt_dim = 50;

    robot->acquire_sensors();
    map_sliders(robot->get_slider_positions(), sliders_zero);
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }

    rt_printf("control loop started \n");

    rt_printf("start calibration \n");

    // Calibrate the robot.
    blmc_robots::Vector12d joint_index_to_zero =
        thread_data_ptr->joint_index_to_zero;

    thread_data_ptr->robot->calibrate(joint_index_to_zero);

    // Run the main program.
    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot->acquire_sensors();

        // acquire the motor enabled signal.
        motor_enabled = robot->get_motor_enabled();

        map_sliders(robot->get_slider_positions(), sliders);

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
            kp * (desired_joint_position - robot->get_joint_positions()) -
            kd * robot->get_joint_velocities();

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
            blmc_robots::Vector12d current_index_to_zero =
                joint_index_to_zero - robot->get_joint_positions();

            // printf("\33[H\33[2J");  // clear screen
            print_vector(" sliders_filt", sliders_filt);
            print_vector(" sliders_zero", sliders_zero);
            print_vector(" sliders_raw ", robot->get_slider_positions());
            print_vector(" des_joint_tau", desired_torque);
            print_vector("     joint_pos", robot->get_joint_positions());
            print_vector("     joint_vel", robot->get_joint_velocities());
            print_vector(" des_joint_pos", desired_joint_position);
            print_vector("zero_joint_pos", current_index_to_zero);
        }
        ++count;

        // Send the current to the motor
        // desired_torque.setZero();
        robot->send_target_joint_torque(desired_torque);
        fflush(stdout);

        real_time_tools::Timer::sleep_sec(0.001);
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_solo12 network_id`.");
    }

    real_time_tools::RealTimeThread thread;
    enable_ctrl_c();

    rt_printf("Please put the robot in zero position.\n");
    rt_printf("\n");
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    std::shared_ptr<Solo12> robot = std::make_shared<Solo12>();
    robot->initialize(argv[1], "does_not_matter");
    robot->set_max_current(4.0);

    ThreadCalibrationData_t thread_data(robot);

    thread.create_realtime_thread(&control_loop, &thread_data);

    rt_printf("control loop started \n");
    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    thread.join();

    return 0;
}
