/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Finger class in a small demo.
 */

#include <realtime_tools/realtime_publisher.h>
#include "real_time_tools/spinner.hpp"
#include "blmc_robots/finger.hpp"

#include "blmc_robots/disentanglement_platform.hpp"

#include <math.h>

// Including ROS related libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blmc_robots/RobotData.h"
#include <iostream>
#include <list>
#include <algorithm>



using namespace blmc_robots;

class Controller
{
public:
  /**
   * @brief Construct a new PDController object.
   *
   * @param motor_slider_pairs
   */


  Controller(std::shared_ptr<DisentanglementPlatform> disentanglement_platform,
             ros::NodeHandle &node):
      disentanglement_platform_(disentanglement_platform),
      realtime_pub_(node, "robot_state", 1000)
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
  //ros::Publisher chatter_pub = node.advertise<blmc_robots::RobotData>("chatter", 1000);

  blmc_robots::RobotData msg;

  std::shared_ptr<DisentanglementPlatform> disentanglement_platform_;
  realtime_tools::RealtimePublisher<blmc_robots::RobotData> realtime_pub_;

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
     * @brief Takes an Eigen::Vector3d and returns an equivalent std::vector.
     */
  std::vector<double> from_vector2d_to_stdvector(const Eigen::Vector2d& v) const
  {
    std::vector<double> ret(v.data(), v.data() + v.rows() * v.cols());
    return ret;
  }

  /**
    * Defining the count values where we publish the joint positions so that listener
    * can store sparsely defined positions for each joint value...
    * First list allows publishing corresponds to the YAW angles(joint A),
    * which completes its period in 10000 steps. We divide it into 20 steps as following.
    * The second corresponds to PITCH angles (joint B).
    */

  std::list<int> publish_intervals={2500,3100,3300,3500,3700,3900,4100,4300,4500,4700,
                                    5300,5500,5700,5900,6100,6300,6500,6700,6900,7500};

  std::list<int> publish_intervals_pitch={2,4,5,6,7,8,9,10,11,12,
                                    15,16,17,18,19,20,21,22,23,25}; // upto 26 because count starts from 1, not 0.

  /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
  void loop()
  {

      double kp = 3.6;
      double kd = 2.3;
      double ki = 0.1;

      real_time_tools::Spinner spinner;
      double interval = 0.001;  //0.001 results in 10,000 steps for full interval
      double interval_b = 0.2;  // 0.5 results in 20 steps for full interval, 0.25 in 40, 0.2 in 50
      spinner.set_period(interval);
      size_t count = 0;
      size_t pitch_count = 0;

      double time_a = 0;
      double time_b = 0;
      double frequency = 0.1;
      double desired_angle_B = 0.0;
      Eigen::Vector2d position_error;
      Eigen::Vector2d total_error;

      position_error = Eigen::Vector2d::Zero();
      total_error = Eigen::Vector2d::Zero();

      while(true)
      {
          double desired_angle_A = (sin(time_a * frequency * 2 * M_PI)) * 0.5 * M_PI; // Yaw

          if ((count) %10000 == 0) // 10000 steps for one period
             {
              desired_angle_B = (cos(time_b * frequency * 2 * M_PI)) * 0.5 * M_PI; // Pitch
              time_b += interval_b;
              count = 0;
              pitch_count++;
              }

        time_a += interval;

        // the slider goes from 0 to 1 so we go from -0.5rad to 0.5rad
        Eigen::Vector2d desired_angles = (disentanglement_platform_->get_slider_positions().array() - 0.5) * 2 * M_PI;

        desired_angles[0] = desired_angle_A;   // Motor A.
        desired_angles[1] = desired_angle_B;   // Motor B.

        position_error = disentanglement_platform_->get_angles() - desired_angles;
        total_error += position_error;
        // we implement here a small pd control at the current level
        Eigen::Vector2d desired_torque = - kp * position_error -
                          kd * disentanglement_platform_->get_angular_velocities(); // -
                          //ki * total_error ;


        // Send the current to the motor
        disentanglement_platform_->set_torques(desired_torque);
        disentanglement_platform_->send_torques();

        // Setting up the values to be published by the publisher
        // Message Publishing. ---------------------------------------------------------

        // Filling in with the measured values.
        msg.joint_angles = from_vector2d_to_stdvector(disentanglement_platform_->get_angles());
        msg.joint_velocities = from_vector2d_to_stdvector(disentanglement_platform_->get_angular_velocities());
        msg.desired_angles = from_vector2d_to_stdvector(desired_angles);
        msg.header.stamp = ros::Time::now();

        bool publish_yaw_joints = (std::find(publish_intervals.begin(), publish_intervals.end(), count) != publish_intervals.end());
        bool publish_pitch_joints = (std::find(publish_intervals_pitch.begin(), publish_intervals_pitch.end(), pitch_count) != publish_intervals_pitch.end());

        if (publish_yaw_joints && publish_pitch_joints)
        {
        if (realtime_pub_.trylock()) {
            realtime_pub_.msg_ = msg;
            realtime_pub_.unlockAndPublish();
        }
        }

        // print -------------------------------------------------------------------
        spinner.spin();

        if ((count % 500) == 0)
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



          Eigen::Vector2d angles = disentanglement_platform_->get_angles();

          rt_printf("positions: [");
          for(int i=0 ; i < angles.size() ; ++i)
          {
            rt_printf("%f, ", angles(i));
          }
          rt_printf("]\n");
        }
        ++count;

      if (pitch_count>26) // This is after collecting 20x20 = 400 images
      {rt_printf("BREAKING LOOP");
          std::exit(EXIT_FAILURE);}

      }//endwhile

  }



  /**
   * @brief managing the stopping of the loop
   */
  bool stop_loop;

}; // end class PDController definition



int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");
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
        motors[0]  = std::make_shared<blmc_drivers::SafeMotor>   (board, 0, 1.6);
        motors[1]  = std::make_shared<blmc_drivers::SafeMotor>   (board, 1, 1.6);

        std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 2> sliders;
        sliders[0] = std::make_shared<blmc_drivers::AnalogSensor>(board, 0);
        sliders[1] = std::make_shared<blmc_drivers::AnalogSensor>(board, 1);

        disentanglement_platform = std::make_shared<DisentanglementPlatform>(motors, sliders);
    }


    ros::NodeHandle n;
    Controller controller(disentanglement_platform, n);


    real_time_tools::Timer::sleep_sec(2.);

    rt_printf("controller is set up \n");

    real_time_tools::block_memory();
    controller.start_loop();
    //real_time_tools::create_realtime_thread(thread, &control_loop, &(*disentanglement_platform));

    rt_printf("control loop started \n");

    while(true)
    {
        real_time_tools::Timer::sleep_sec(0.0001);
    }

    return 0;
}
