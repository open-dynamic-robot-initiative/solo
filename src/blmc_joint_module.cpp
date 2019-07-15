/**
 * @file blmc_joint_module.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @author Manuel Wuthrich
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-07-11
 */

#include <cmath>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/iostream.hpp"
#include "blmc_robots/blmc_joint_module.hpp"

namespace blmc_robots{


BlmcJointModule::BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle)
{
    motor_ = motor;
    motor_constant_ = motor_constant;
    gear_ratio_ = gear_ratio;
    set_zero_angle(zero_angle);
}

void BlmcJointModule::set_torque(const double& desired_torque)
{
    double desired_current = joint_torque_to_motor_current(desired_torque);

    if(std::fabs(desired_current) > 2.1)
    {
        std::cout << "something went wrong, it should never happen"
                      "that desired_current > 2.1. desired_current: "
                  << desired_current;
        exit(-1);
    }
    motor_->set_current_target(desired_current);
}

void BlmcJointModule::set_zero_angle(const double& zero_angle)
{
    zero_angle_ = zero_angle;
}


void BlmcJointModule::send_torque()
{
    motor_->send_if_input_changed();
}

double BlmcJointModule::get_sent_torque() const
{
    auto measurement_history =
            motor_->get_sent_current_target();

    if(measurement_history->length() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return motor_current_to_joint_torque(measurement_history->newest_element());
}

double BlmcJointModule::get_measured_torque() const
{
    return motor_current_to_joint_torque(get_motor_measurement(mi::current));
}

double BlmcJointModule::get_measured_angle() const
{
    return get_motor_measurement(mi::position) / gear_ratio_ - zero_angle_;
}

double BlmcJointModule::get_measured_velocity() const
{
    return get_motor_measurement(mi::velocity) / gear_ratio_;
}

double BlmcJointModule::joint_torque_to_motor_current(double torque) const
{
    return torque / gear_ratio_ / motor_constant_;
}

double BlmcJointModule::motor_current_to_joint_torque(double current) const
{
    return current *  gear_ratio_ * motor_constant_;
}

double BlmcJointModule::get_index_angle() const
{
    return get_motor_measurement(mi::encoder_index) / gear_ratio_;
}

double BlmcJointModule::get_zero_angle() const
{
    return zero_angle_;
}

double BlmcJointModule::get_motor_measurement(const mi& measurement_id) const
{
    auto measurement_history =
            motor_->get_measurement(measurement_id);

    if(measurement_history->length() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return measurement_history->newest_element();
}

long int BlmcJointModule::get_motor_measurement_index(const mi& measurement_id) const
{
    auto measurement_history =
            motor_->get_measurement(measurement_id);

    if(measurement_history->length() == 0)
    {
        return std::numeric_limits<long int>::quiet_NaN();
    }
    return measurement_history->newest_timeindex();
}

bool BlmcJointModule::calibrate(double& angle_zero_to_index,
                                double& index_angle,
                                bool mechanical_calibration)
{
    // reset the ouput
    index_angle = 0.0;

    // we reset the internal zero angle.
    zero_angle_ = 0.0;

    // Small D gain
    double k_d = 0.01;
    // Small desired velocity
    double joint_vel_des = 0.05;

    
    long int last_index_time = get_motor_measurement_index(mi::encoder_index);
    if(std::isnan(last_index_time)){last_index_time = -1;}
    bool reached_next_index = false;
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    rt_printf("Search for the index\n");
    while(!reached_next_index)
    {
      // Velocity controller
      double torque = k_d * (joint_vel_des - get_measured_velocity());
      // Send the torque command
      set_torque(torque);
      send_torque();
      // check stop
      long int actual_index_time = get_motor_measurement_index(mi::encoder_index);
      double actual_index_angle = get_motor_measurement(mi::encoder_index);

      reached_next_index = (actual_index_time > last_index_time);
      rt_printf("last_index_time=%ld, actual_index_time=%ld, actual_index_angle=%f\n", last_index_time, actual_index_time, actual_index_angle);
      if(reached_next_index)
      {
          index_angle = actual_index_angle;
      }
      spinner.spin();
    }

    rt_printf("Stop the motion\n");
    bool stopped = false;
    while(!stopped)
    {
      // Velocity controller
      double torque = k_d * (joint_vel_des - get_measured_velocity());
      // Send the torque command
      set_torque(torque);
      send_torque();
      // check stop
      stopped = get_measured_velocity() < 0.001;
      spinner.spin();
    }

    // reset the control to zero torque
    set_torque(0.0);
    send_torque();
    spinner.spin();

    // get the indexes and stuff
    if(mechanical_calibration)
    {
        angle_zero_to_index = index_angle;
    }
    zero_angle_ = index_angle - angle_zero_to_index;

    // rt_printf("Position Control\n")
    // // Go to 0
    // double torque_int = 0.0;
    // double torque_sat = 0.2; // Nm
    // bool reached_zero_pose = 0;
    // while(!reached_zero_pose)
    // {
    //   // Small P gain
    //   double k_p = 0.01;
    //   // small I gain
    //   double k_i = 0.001;
    //   // compute the error 
    //   double err = - get_measured_angle();
    //   // small saturation in intensity
    //   torque_int += k_i * err * 0.001; // 1 ms sampling period
    //   if(torque_int > torque_sat){torque_int = torque_sat;}
    //   if(torque_int < -torque_sat){torque_int = -torque_sat;}
    //   // Position controller
    //   double torque = k_p * err + torque_int ;
    //   // Send the torque command
    //   set_torque(torque);
    //   send_torque();
    //   // check out
    //   reached_zero_pose = (err <= 1e-3); // nearly 0.1 degree
    //   spinner.spin();
    // }
    // reset the control to zero torque
    set_torque(0.0);
    send_torque();
    spinner.spin();
}

} // namespace