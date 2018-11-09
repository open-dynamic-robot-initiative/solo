/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Finger class in a small demo.
 */

#include "blmc_robots/finger/finger.hpp"

using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
  Finger& robot = *(static_cast<Finger*>(robot_void_ptr));

  double kp = 0.2;
  double kd = 0.04;

  Timer<10> time_logger("controller");
  while(true)
  {
    // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
    Eigen::Vector3d desired_angles = (robot.get_slider_positions().array() - 0.5) * 2 * M_PI;

    // we implement here a small pd control at the current level
    Eigen::Vector3d desired_torque = kp * (desired_angles - robot.get_angles()) -
                      kd * robot.get_angular_velocities();

    // Send the current to the motor
    robot.send_torques(desired_torque);

    // print -----------------------------------------------------------
    Timer<>::sleep_ms(1);

    time_logger.end_and_start_interval();
    if ((time_logger.count() % 1000) == 0)
    {
      rt_printf("sending currents: [");
      for(int i=0 ; i<desired_torque.size() ; ++i)
      {
        rt_printf("%f, ", desired_torque(i));
      }
      rt_printf("]\n");


      Eigen::Vector3d angles = robot.get_angles();

      rt_printf("positions: [");
      for(int i=0 ; i < angles.size() ; ++i)
      {
        rt_printf("%f, ", angles(i));
      }
      rt_printf("]\n");
    }
  }//endwhile
}// end control_loop

int main(int argc, char **argv)
{
  Finger robot;

  robot.initialize();

  rt_printf("controller is set up \n");

  osi::start_thread(&control_loop, &robot);

  rt_printf("control loop started \n");

  while(true)
  {
    Timer<>::sleep_ms(10);
  }

  return 0;
}
