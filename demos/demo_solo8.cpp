/**
 * \file demo_solo8.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Solo8 class in a small demo.
 */

#include <numeric>
#include "solo/common_programs_header.hpp"
#include "solo/solo8.hpp"
#include "common_demo_header.hpp"

using namespace solo;

typedef ThreadCalibrationData<Solo8> ThreadCalibrationData_t;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr)
{
    ThreadCalibrationData_t* thread_data_ptr =
        (static_cast<ThreadCalibrationData_t*>(thread_data_void_ptr));

    std::shared_ptr<Solo8> robot = thread_data_ptr->robot;

    double kp = 3.0;
    double kd = 0.05;
    double max_range = M_PI;
    Vector8d desired_joint_position;
    Vector8d desired_torque;

    Eigen::Vector4d sliders;
    Eigen::Vector4d sliders_filt;
    Eigen::Vector4d sliders_init;

    std::vector<std::deque<double> > sliders_filt_buffer(
        robot->get_slider_positions().size());
    size_t max_filt_dim = 100;
    for (unsigned i = 0; i < sliders_filt_buffer.size(); ++i)
    {
        sliders_filt_buffer[i].clear();
    }

    robot->acquire_sensors();
    sliders_init = robot->get_slider_positions();


    // Calibrates the robot.
    solo::Vector8d joint_index_to_zero = thread_data_ptr->joint_index_to_zero;
    robot->request_calibration(joint_index_to_zero);

    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot->acquire_sensors();

        // aquire the slider signal
        sliders = robot->get_slider_positions();
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
                max_range * (sliders_filt(0) - sliders_init(0));
            desired_joint_position(2 * i + 1) =
                -2. * max_range * (sliders_filt(0) - sliders_init(0));
        }

        desired_joint_position.tail(4) *= -1;

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot->get_joint_positions()) -
            kd * robot->get_joint_velocities();

        // Send the current to the motor
        robot->send_target_joint_torque(desired_torque);

        real_time_tools::Timer::sleep_sec(0.001);

        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            solo::Vector8d current_index_to_zero =
                joint_index_to_zero - robot->get_joint_positions();

            print_vector("des_joint_tau", desired_torque);
            print_vector("des_joint_tau", desired_torque);
            print_vector("    joint_pos", robot->get_joint_positions());
            print_vector("des_joint_pos", desired_joint_position);
            print_vector("   slider_pos", robot->get_slider_positions());
            print_vector("zero_joint_pos", current_index_to_zero);
        }
        ++count;
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    enable_ctrl_c();

    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_solo8 network_id`.");
    }

    real_time_tools::RealTimeThread thread;
    enable_ctrl_c();

    rt_printf("Please put the robot in zero position.\n");
    rt_printf("\n");
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    std::shared_ptr<Solo8> robot = std::make_shared<Solo8>();
    robot->initialize(std::string(argv[1]));

    ThreadCalibrationData_t thread_data(robot);
    thread.create_realtime_thread(&control_loop, &thread_data);

    rt_printf("control loop started \n");

    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    thread.join();

    return 0;
}
