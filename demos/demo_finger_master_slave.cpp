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


class Controller
{
public:
  /**
   * @brief Construct a new PDController object.
   *
   * @param motor_slider_pairs
   */
  Controller(std::shared_ptr<Finger> slave_finger,
             std::shared_ptr<Finger> master_finger):
      slave_finger_(slave_finger),
      master_finger_(master_finger)
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

  std::shared_ptr<Finger> slave_finger_;
  std::shared_ptr<Finger> master_finger_;


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

      double kp = 1.2;
      double kd = 0.04;

      real_time_tools::Spinner spinner;
      spinner.set_period(0.001);
      size_t count = 0;
      while(true)
      {
        // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
        Eigen::Vector3d desired_angles = (slave_finger_->get_slider_positions().array() - 0.5) * 2 * M_PI;
        desired_angles = master_finger_->get_angles();
        desired_angles(2) = - desired_angles(2);


        // we implement here a small pd control at the current level
        Eigen::Vector3d desired_torque = kp * (desired_angles - slave_finger_->get_angles()) -
                          kd * slave_finger_->get_angular_velocities();


        // Send the current to the motor
        slave_finger_->set_torques(desired_torque);
        slave_finger_->send_torques();

        desired_torque(2) = - desired_torque(2);
        master_finger_->set_torques(-desired_torque);
        master_finger_->send_torques();


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

          rt_printf("desired positions: [");
          for(int i=0 ; i < desired_angles.size() ; ++i)
          {
            rt_printf("%f, ", desired_angles(i));
          }
          rt_printf("]\n");



          Eigen::Vector3d angles = slave_finger_->get_angles();

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
    std::shared_ptr<Finger> slave_finger;
    {
        // initialize the communication with the can cards
        auto can_bus_0 = std::make_shared<blmc_drivers::CanBus>("can1");
        auto can_bus_1 = std::make_shared<blmc_drivers::CanBus>("can3");

        // get all informatino about the control cards
        auto board_0 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_0);
        auto board_1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_1);

        // get the drivers for the motors and the sliders
        // two individual motors on individual leg style mounting on the left of the
        // table
        std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> motors;
        motors[0]  = std::make_shared<blmc_drivers::SafeMotor>   (board_0, 0, 2.0);
        motors[1]  = std::make_shared<blmc_drivers::SafeMotor>   (board_0, 1, 2.0);
        motors[2]  = std::make_shared<blmc_drivers::SafeMotor>   (board_1, 0, 2.0);

        std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3> sliders;
        sliders[0] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 0);
        sliders[1] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 1);
        sliders[2] = std::make_shared<blmc_drivers::AnalogSensor>(board_1, 0);

        slave_finger = std::make_shared<Finger>(motors, sliders);
    }


    // master finger ------------------------------------------------------------
    std::shared_ptr<Finger> master_finger;
    {
        // initialize the communication with the can cards
        auto can_bus_0 = std::make_shared<blmc_drivers::CanBus>("can6");
        auto can_bus_1 = std::make_shared<blmc_drivers::CanBus>("can7");

        // get all informatino about the control cards
        auto board_0 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_0);
        auto board_1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_1);

        // get the drivers for the motors and the sliders
        // two individual motors on individual leg style mounting on the left of the
        // table
        std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> motors;
        motors[0]  = std::make_shared<blmc_drivers::SafeMotor>   (board_0, 0, 2.0);
        motors[1]  = std::make_shared<blmc_drivers::SafeMotor>   (board_0, 1, 2.0);
        motors[2]  = std::make_shared<blmc_drivers::SafeMotor>   (board_1, 0, 2.0);

        std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3> sliders;
        sliders[0] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 0);
        sliders[1] = std::make_shared<blmc_drivers::AnalogSensor>(board_0, 1);
        sliders[2] = std::make_shared<blmc_drivers::AnalogSensor>(board_1, 0);

        master_finger = std::make_shared<Finger>(motors, sliders);
    }

    Controller controller(slave_finger, master_finger);




    real_time_tools::Timer::sleep_sec(0.11);

    rt_printf("controller is set up \n");

    real_time_tools::block_memory();
    controller.start_loop();
//    real_time_tools::create_realtime_thread(thread, &control_loop, &(*slave_finger));

    rt_printf("control loop started \n");

    while(true)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    return 0;
}
