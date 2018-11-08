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

  double kp = 1;
  double kd = 0.2;
  Eigen::Vector3d desired_motor_position;
  Eigen::Vector3d desired_torque;

  Eigen::Vector3d debug_des_pd;
  Eigen::Vector3d debug_des_u_sat;
  Eigen::Vector3d debug_des_l_sat;

  Timer<10> time_logger("controller");
  while(true)
  {
    // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
    desired_motor_position = (robot.get_slider_positions().array() - 0.5);

    // we implement here a small pd control at the current level
    desired_torque = kp * (desired_motor_position - robot.get_positions()) -
                      kd * robot.get_velocities();

    // Saturate the desired current
    desired_torque = desired_torque.array().min(0.2 * Eigen::Vector3d::Ones().array());
    desired_torque = desired_torque.array().max(-0.2 * Eigen::Vector3d::Ones().array());

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
