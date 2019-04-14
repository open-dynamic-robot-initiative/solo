/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the RealFinger class in a small demo.
 */

#include "real_time_tools/spinner.hpp"
#include "blmc_robots/real_finger.hpp"

#include "blmc_robots/disentanglement_platform.hpp"

#include <math.h>

using namespace blmc_robots;


class Controller
{
public:
  /**
   * @brief Construct a new PDController object.
   *
   * @param motor_slider_pairs
   */
  Controller(std::shared_ptr<DisentanglementPlatform> disentanglement_platform):
      disentanglement_platform_(disentanglement_platform)
  {
    stop_loop=false;
  }

  /**
   * @brief Destroy the PDController object
   */
  ~Controller()
  {
    stop_loop=true;
    real_time_tools::join_thread(rt_thread_);
  }

  /**
   * @brief This method is a helper to start the thread loop.
   */
  void start_loop()
  {
    real_time_tools::create_realtime_thread(
          rt_thread_, &Controller::loop, this);
  }

private:

  std::shared_ptr<DisentanglementPlatform> disentanglement_platform_;

  /**
   * @brief This is a pair of motor and sliders so that we associate one with
   * the other.
   */
  /**
   * @brief This is the real time thread object.
   */
  real_time_tools::RealTimeThread rt_thread_;

  /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
  static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
  {
    ((Controller*)(instance_pointer))->loop();
  }

  /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
  void loop()
  {

      double kp = 0.5;
      double kd = 0.0025;

      real_time_tools::Spinner spinner;
      double interval = 0.001;
      spinner.set_period(interval);
      size_t count = 0;

      double time = 0;
      double frequency = 0.2;
      while(true)
      {
          double desired_angle = (sin(time * frequency * 2 * M_PI)) * 0.5 * M_PI;


          time += interval;

        // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
        Eigen::Vector2d desired_angles = (disentanglement_platform_->get_slider_positions().array() - 0.5) * 2 * M_PI;

        desired_angles[0] = desired_angle;
        desired_angles[1] = desired_angle;


        // we implement here a small pd control at the current level
        Eigen::Vector2d desired_torque = kp * (desired_angles - disentanglement_platform_->get_measured_angles()) -
                          kd * disentanglement_platform_->get_measured_velocities();


        // Send the current to the motor
        disentanglement_platform_->set_torques(desired_torque);
        disentanglement_platform_->send_torques();


        // print -------------------------------------------------------------------
        spinner.spin();

        if ((count % 1000) == 0)
        {
          rt_printf("sending torques: [");
          for(int i=0 ; i<desired_torque.size() ; ++i)
          {
            rt_printf("%f, ", desired_torque(i));
          }
          rt_printf("]\n");

          rt_printf("desired angles: [");
          for(int i=0 ; i < desired_angles.size() ; ++i)
          {
            rt_printf("%f, ", desired_angles(i));
          }
          rt_printf("]\n");



          Eigen::Vector2d angles = disentanglement_platform_->get_measured_angles();

          rt_printf("positions: [");
          for(int i=0 ; i < angles.size() ; ++i)
          {
            rt_printf("%f, ", angles(i));
          }
          rt_printf("]\n");
        }
        ++count;
      }//endwhile

  }

  /**
   * @brief managing the stopping of the loop
   */
  bool stop_loop;

}; // end class PDController definition



int main(int argc, char **argv)
{
    real_time_tools::RealTimeThread thread;
    osi::initialize_realtime_printing();

    // slave finger ------------------------------------------------------------
    std::shared_ptr<DisentanglementPlatform> disentanglement_platform;
    {
        // initialize the communication with the can cards
        auto can_bus = std::make_shared<blmc_drivers::CanBus>("can0");

        // get all informatino about the control cards
        auto board = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus);

        // get the drivers for the motors and the sliders
        // two individual motors on individual leg style mounting on the left of the
        // table
        std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 2> motors;
        motors[0]  = std::make_shared<blmc_drivers::SafeMotor>   (board, 0, 1.0);
        motors[1]  = std::make_shared<blmc_drivers::SafeMotor>   (board, 1, 1.0);

        std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 2> sliders;
        sliders[0] = std::make_shared<blmc_drivers::AnalogSensor>(board, 0);
        sliders[1] = std::make_shared<blmc_drivers::AnalogSensor>(board, 1);

        disentanglement_platform = std::make_shared<DisentanglementPlatform>(motors, sliders);
    }



    Controller controller(disentanglement_platform);




    real_time_tools::Timer::sleep_sec(5.);

    rt_printf("controller is set up \n");

    real_time_tools::block_memory();
    controller.start_loop();
//    real_time_tools::create_realtime_thread(thread, &control_loop, &(*disentanglement_platform));

    rt_printf("control loop started \n");

    while(true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    return 0;
}
