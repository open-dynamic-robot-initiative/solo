/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include <Eigen/Eigen>
#include <deque>
#include <numeric>
#include <cmath>
#include "blmc_robots/teststand.hpp"
#include "real_time_tools/timer.hpp"

using namespace blmc_robots;

void print_vector(std::string v_name, Eigen::Ref<Eigen::VectorXd> v)
{
  v_name += ": [";
  rt_printf("%s", v_name.c_str());
  for(int i=0 ; i<v.size() ; ++i)
  {
    rt_printf("%f, ", v(i));
  }
  rt_printf("]\n");
}

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
  Teststand& robot = *(static_cast<Teststand*>(robot_void_ptr));

  double kp = 5.0 ;
  double kd = 1.0 ;
  double max_range = M_PI;
  Vector2d desired_joint_position;
  Vector2d desired_torque;

  Teststand::VectorSlider sliders;
  Teststand::VectorSlider sliders_filt;

  std::vector<std::deque<double> > sliders_filt_buffer(
        robot.get_slider_positions().size());
  int max_filt_dim = 200;
  for(unsigned i=0 ; i<sliders_filt_buffer.size() ; ++i)
  {
    sliders_filt_buffer[i].clear();
  }
  size_t count = 0;
  while(true)
  {
    // acquire the sensors
    robot.acquire_sensors();

    // aquire the slider signal
    sliders = robot.get_slider_positions();
    // filter it
    for(unsigned i=0 ; i<sliders_filt_buffer.size() ; ++i)
    {
      if(sliders_filt_buffer[i].size() >= max_filt_dim)
      {
        sliders_filt_buffer[i].pop_front();
      }
      sliders_filt_buffer[i].push_back(sliders(i));
      sliders_filt(i) = std::accumulate(sliders_filt_buffer[i].begin(),
                                        sliders_filt_buffer[i].end(), 0.0)/
                        (double)sliders_filt_buffer[i].size();
    }

    // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
    for(unsigned i=0 ; i < 1 ; ++i)
    {
      //desired_pose[i].push_back
      desired_joint_position(2 * i + 0) = max_range * (sliders_filt(i) - 0.5);
      desired_joint_position(2 * i + 1) = -2 * max_range * (sliders_filt(i) - 0.5);
    }

    // we implement here a small pd control at the current level
    desired_torque = kp * (desired_joint_position - robot.get_joint_positions())
                     - kd * robot.get_joint_velocities();

    // // Send the current to the motor
    robot.send_target_joint_torque(desired_torque);

    // print -----------------------------------------------------------
    real_time_tools::Timer::sleep_sec(0.001);

    if ((count % 1000) == 0)
    {
      print_vector("des_joint_tau  ", desired_torque);
      print_vector("    joint_pos  ", robot.get_joint_positions());
      print_vector("des_joint_pos  ", desired_joint_position);
      print_vector("ati_force      ", robot.get_ati_force());
      print_vector("ati_torque     ", robot.get_ati_torque());
      print_vector("contact_sensors", robot.get_contact_sensors_states());
      print_vector("height_sensors ", robot.get_height_sensors());
    }
    ++count;
  }//endwhile
}// end control_loop

int main(int argc, char **argv)
{
  real_time_tools::RealTimeThread thread;

  Teststand robot;

  robot.initialize();

  rt_printf("controller is set up \n");

  real_time_tools::create_realtime_thread(thread, &control_loop, &robot);

  rt_printf("control loop started \n");

  while(true)
  {
      real_time_tools::Timer::sleep_sec(0.01);
  }

  return 0;
}
