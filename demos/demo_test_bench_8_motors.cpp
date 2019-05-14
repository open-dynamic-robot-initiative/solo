/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include <signal.h>
#include <atomic>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/thread.hpp"
#include "blmc_robots/test_bench_8_motors.hpp"

using namespace blmc_robots;

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos (false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 * 
 * @param s 
 */
void my_handler(int s){
  StopDemos = true;
}

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
  TestBench8Motors& robot = *(static_cast<TestBench8Motors*>(robot_void_ptr));

  double kp = 5;
  double kd = 1;
  Vector8d desired_motor_position;
  Vector8d desired_current;

  Vector8d debug_des_pd;
  Vector8d debug_des_u_sat;
  Vector8d debug_des_l_sat;

  real_time_tools::Spinner spinner;
  spinner.set_period(0.001);
  size_t count=0;
  while(!StopDemos)
  {
    // acquire the sensors
    robot.acquire_sensors();

    // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
    desired_motor_position = robot.get_max_range() *
                             (robot.get_slider_positions().array() - 0.5);

    // we implement here a small pd control at the current level
    desired_current = kp * (desired_motor_position - robot.get_motor_positions()) -
                      kd * robot.get_motor_velocities();

    // Saturate the desired current
    desired_current = desired_current.array().min(robot.get_max_current() * Vector8d::Ones().array());
    desired_current = desired_current.array().max(-robot.get_max_current() * Vector8d::Ones().array());

    // Send the current to the motor
    robot.send_target_current(desired_current);

    // print -----------------------------------------------------------
    spinner.spin();
    if ((count % 1000) == 0)
    {
      rt_printf("sending currents: [");
      for(int i=0 ; i<desired_current.size() ; ++i)
      {
        rt_printf("%f, ", desired_current(i));
      }
      rt_printf("]\n");
    }
    ++count;
  }//endwhile
  return THREAD_FUNCTION_RETURN_VALUE;
}// end control_loop

int main(int argc, char **argv)
{
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  StopDemos = false;

  TestBench8Motors robot;

  robot.initialize();

  rt_printf("controller is set up \n");

  real_time_tools::RealTimeThread rt_thread;
  
  rt_thread.create_realtime_thread(&control_loop, &robot);

  rt_printf("control loop started \n");

  // Wait until the application is killed.
  while(!StopDemos)
  {
    real_time_tools::Timer::sleep_sec(0.01);
  }
  rt_thread.join();

  return 0;
}
