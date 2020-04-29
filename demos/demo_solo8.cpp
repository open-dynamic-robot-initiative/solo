/**
 * \file demo_solo8.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo8 class in a small demo.
 */

#include <numeric>
#include "blmc_robots/solo8.hpp"
#include "blmc_robots/common_programs_header.hpp"


using namespace blmc_robots;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Solo8& robot = *(static_cast<Solo8*>(robot_void_ptr));

    double kp = 3.0;
    double kd = 0.05;
    double max_range = M_PI;
    Vector8d desired_joint_position;
    Vector8d desired_torque;

    Eigen::Vector4d sliders;
    Eigen::Vector4d sliders_filt;
    Eigen::Vector4d sliders_init;

    std::vector<std::deque<double> > sliders_filt_buffer(
        robot.get_slider_positions().size());
    size_t max_filt_dim = 100;
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }

    robot.acquire_sensors();
    sliders_init = robot.get_slider_positions();

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
            desired_joint_position(2 * i) = max_range * (sliders_filt(0) - sliders_init(0));
            desired_joint_position(2 * i + 1) =
                -2. * max_range * (sliders_filt(0) - sliders_init(0));
        }

        desired_joint_position.tail(4) *= -1;

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();

        // Send the current to the motor
        robot.send_target_joint_torque(desired_torque);

        // print -----------------------------------------------------------
        real_time_tools::Timer::sleep_sec(0.001);

        if ((count % 250) == 0)
        {
            print_vector("des_joint_tau", desired_torque);
            print_vector("des_joint_tau", desired_torque);
            print_vector("    joint_pos", robot.get_joint_positions());
            print_vector("des_joint_pos", desired_joint_position);
            print_vector("   slider_pos", robot.get_slider_positions());
        }
        ++count;
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    enable_ctrl_c();

    if(argc != 2)
    {
       throw std::runtime_error("Wrong number of argument: `./demo_solo8 network_id`.");
    }

    real_time_tools::RealTimeThread thread;

    Solo8 robot;
    robot.initialize(std::string(argv[1]));

    thread.create_realtime_thread(&control_loop, &robot);
    // thread.create_realtime_thread(&control_loop);

    rt_printf("control loop started \n");

    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    thread.join();

    return 0;
}
