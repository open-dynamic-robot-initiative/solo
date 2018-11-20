/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include <Eigen/Eigen>
#include "blmc_robots/quadruped.hpp"
#include <deque>

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
  Quadruped& robot = *(static_cast<Quadruped*>(robot_void_ptr));

  double kp = 5;
  double kd = 1;
  double max_range = 14.0*2.0;
  Vector8d desired_motor_position;
  Vector8d desired_current;

  Vector8d debug_des_pd;
  Vector8d debug_des_u_sat;
  Vector8d debug_des_l_sat;

//  std::vector<std::deque<double> > desired_pose(4);
//  for(unsigned i=0 ; i<desired_pose.size() ; ++i)
//  {
//    desired_pose.clear();
//    desired_pose.reserve(200);
//  }

  Timer<10> time_logger("controller");
  while(true)
  {
    // acquire the sensors
    robot.acquire_sensors();

    // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
    for(unsigned i=0 ; i<4 ; ++i)
    {
      //desired_pose[i].push_back
      desired_motor_position(2*i) =
          max_range * (robot.get_slider_positions()(i) - 0.5);
      desired_motor_position(2*i+1) =
          max_range * (robot.get_slider_positions()(i) - 0.5);
    }


    // we implement here a small pd control at the current level
    desired_current = kp * (desired_motor_position - robot.get_motor_positions()) -
                      kd * robot.get_motor_velocities();

    // Saturate the desired current
    desired_current = desired_current.array().min(
                        robot.get_max_current().array());
    desired_current = desired_current.array().max(
                        -robot.get_max_current().array());

    // Send the current to the motor
    robot.send_target_current(desired_current);

    // print -----------------------------------------------------------
    Timer<>::sleep_ms(1);

    time_logger.end_and_start_interval();
    if ((time_logger.count() % 1000) == 0)
    {
      print_vector("desired_current", desired_current);
      print_vector("slider", robot.get_slider_positions());
      print_vector("contact_states", robot.get_contact_sensors_states());
      print_vector("joint_pos", robot.get_joint_positions());
      print_vector("motor_pos", robot.get_motor_positions());
    }
  }//endwhile
}// end control_loop

int main(int argc, char **argv)
{
  Quadruped robot;

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

