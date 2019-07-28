/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Teststand class in a small demo.
 */

// Exiting on ctrl+c
#include <signal.h>
#include <atomic>

#include <Eigen/Eigen>
#include <deque>
#include <numeric>
#include <cmath>
#include "blmc_robots/teststand.hpp"
#include "real_time_tools/timer.hpp"

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

  std::array<double, 2> zero_to_index_angle;
  std::array<double, 2> index_angle;
  bool mechanical_calibration;

  zero_to_index_angle.fill(0.0);
  index_angle.fill(0.0);
  mechanical_calibration = true;

  robot.calibrate(zero_to_index_angle, index_angle, mechanical_calibration);

  for(unsigned i=0 ; i<2 ; ++i)
  {
    rt_printf("zero_to_index_angle[%d] = %f\n", i, zero_to_index_angle[i]);
    rt_printf("index_angle[%d] = %f\n", i, index_angle[i]);
  }

  StopDemos = true;

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

  real_time_tools::RealTimeThread thread;

  Teststand robot;

  robot.initialize();
  
  rt_printf("controller is set up \n");
  rt_printf("Press enter to launch the calibration \n");
  char str[256];
  std::cin.get (str,256);    // get c-string

  thread.create_realtime_thread(&control_loop, &robot);

  // Wait until the application is killed.
  thread.join();

  rt_printf("Exit cleanly \n");

  return 0;
}
