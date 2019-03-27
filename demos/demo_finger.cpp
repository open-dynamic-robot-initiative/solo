/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Finger class in a small demo.
 */

#include "real_time_tools/spinner.hpp"
#include "blmc_robots/finger.hpp"

using namespace blmc_robots;


static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
  Finger& robot = *(static_cast<Finger*>(robot_void_ptr));

  double kp = 0.2;
  double kd = 0.0025;

  real_time_tools::Spinner spinner;
  spinner.set_period(0.001);
  size_t count = 0;
  while(true)
  {
    // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
    Eigen::Vector3d desired_angles = (robot.get_slider_positions().array() - 0.5) * 2 * M_PI;

    // we implement here a small pd control at the current level
    Eigen::Vector3d desired_torque = kp * (desired_angles - robot.get_angles()) -
                      kd * robot.get_angular_velocities();

    // Send the current to the motor
    robot.set_torques(desired_torque);
    robot.send_torques();


    // print -------------------------------------------------------------------
    spinner.spin();

    if ((count % 1000) == 0)
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
    ++count;
  }//endwhile
}// end control_loop

int main(int argc, char **argv)
{
    // initialize the communication with the can cards
    auto can_bus_0 = std::make_shared<blmc_drivers::CanBus>("can0");
    auto can_bus_1 = std::make_shared<blmc_drivers::CanBus>("can1");

    // get all informatino about the control cards
    auto board_0 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_0);
    auto board_1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_1);

    // get the drivers for the motors and the sliders
    // two individual motors on individual leg style mounting on the left of the
    // table
    std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> motors;
    double max_motor_velocity = 20;
    motors[0]  = std::make_shared<blmc_drivers::SafeMotor>(board_0, 0, 2.0,
        1000, max_motor_velocity);
    motors[1]  = std::make_shared<blmc_drivers::SafeMotor>(board_0, 1, 2.0,
        1000, max_motor_velocity);
    motors[2]  = std::make_shared<blmc_drivers::SafeMotor>(board_1, 0, 2.0,
        1000, max_motor_velocity);

    std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3> sliders;
    sliders[0] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 0);
    sliders[1] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 1);
    sliders[2] = std::make_shared<blmc_drivers::AnalogSensor>(board_1, 0);

    Finger finger(motors, sliders);

    board_0->wait_until_ready();
    board_1->wait_until_ready();

    real_time_tools::block_memory();
    real_time_tools::RealTimeThread thread;
    real_time_tools::create_realtime_thread(thread, &control_loop, &finger);

    rt_printf("control loop started \n");

    while(true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    return 0;
}
