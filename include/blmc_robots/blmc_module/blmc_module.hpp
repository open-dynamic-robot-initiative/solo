#pragma once

#include <Eigen/Eigen>
#include <blmc_robots/common_header.hh>
#include <math.h>
#include <blmc_drivers/devices/motor.hpp>


namespace blmc_robots
{

class BlmcModule
{
public:

    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;


    BlmcModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
               const double& motor_constant = 1.0,
               const double& gear_ratio = 1.0,
               const double& zero_position = 0.0)
    {
        motor_ = motor;

        motor_constant_ = motor_constant;
        gear_ratio_ = gear_ratio;
        zero_angle_ = zero_position;
    }

    void set_torque(const double& desired_torque)
    {
        motor_->set_current_target(desired_torque
                                           / gear_ratio_ / motor_constant_);
    }

    void send_torque()
    {
        motor_->send_if_input_changed();
    }


    double get_torque() const
    {

        return get_motor_measurement(mi::current)
                * gear_ratio_ * motor_constant_;
    }

    double get_angle() const
    {
        return get_motor_measurement(mi::position) / gear_ratio_ * 2 * M_PI
                    - zero_angle_;
    }

    double get_angular_velocity() const
    {
        return get_motor_measurement(mi::velocity) / gear_ratio_ * 2 * M_PI;

    }

    double get_index_angle() const
    {
        return get_motor_measurement(mi::encoder_index) / gear_ratio_ * 2 * M_PI;
    }


private:
    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    double get_motor_measurement(const int& measurement_index) const
    {
        auto measurement_history =
                motor_->get_measurement(measurement_index);

        if(measurement_history->length() == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return measurement_history->newest_element();
    }

    std::shared_ptr<blmc_drivers::MotorInterface> motor_;

    double motor_constant_;

    double gear_ratio_;
    double zero_angle_;
};

} // namespace blmc_robots
