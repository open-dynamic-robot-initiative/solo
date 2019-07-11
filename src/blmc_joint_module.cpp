///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2017-2019, New York University and Max Planck Gesellshaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////


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

double BlmcJointModule::get_motor_measurement(const mi& measurement_index) const
{
    auto measurement_history =
            motor_->get_measurement(measurement_index);

    if(measurement_history->length() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return measurement_history->newest_element();
}

} // namespace