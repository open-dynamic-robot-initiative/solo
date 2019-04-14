#pragma once

#include <iostream>
#include <array>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <blmc_drivers/devices/motor.hpp>
#include <stdexcept>


namespace blmc_robots
{

class BlmcJointModule
{
public:

    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;


    BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle)
    {
        motor_ = motor;
        motor_constant_ = motor_constant;
        gear_ratio_ = gear_ratio;
        set_zero_angle(zero_angle);
    }

    void set_torque(const double& desired_torque)
    {
        double desired_current = torque_to_current(desired_torque);

        if(desired_current > 2.1)
        {
            std::cout << "something went wrong, it should never happen"
                         "that desired_current > 2.1. desired_current: "
                      << desired_current;
            exit(-1);
        }
        motor_->set_current_target(desired_current);
    }

    void set_zero_angle(const double& zero_angle)
    {
        zero_angle_ = zero_angle;
    }


    void send_torque()
    {
        motor_->send_if_input_changed();
    }

    double get_sent_torque() const
    {
        auto measurement_history =
                motor_->get_sent_current_target();

        if(measurement_history->length() == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return current_to_torque(measurement_history->newest_element());
    }

    double get_measured_torque() const
    {
        return current_to_torque(get_motor_measurement(mi::current));
    }

    double get_measured_angle() const
    {
        return get_motor_measurement(mi::position) / gear_ratio_ - zero_angle_;
    }

    double get_measured_velocity() const
    {
        return get_motor_measurement(mi::velocity) / gear_ratio_;
    }

    double torque_to_current(double torque) const
    {
        return torque / gear_ratio_ / motor_constant_;
    }

    double current_to_torque(double current) const
    {
        return current *  gear_ratio_ * motor_constant_;
    }

private:
    double get_index_angle() const
    {
        return get_motor_measurement(mi::encoder_index) / gear_ratio_;
    }

    double get_zero_angle() const
    {
        return zero_angle_;
    }

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




template <int COUNT>
class BlmcJointModules
{
public:
    typedef Eigen::Matrix<double, COUNT, 1> Vector;


    BlmcJointModules(
            const std::array<std::shared_ptr<blmc_drivers::MotorInterface>,
            COUNT>& motors,
            const Vector& motor_constants,
            const Vector& gear_ratios,
            const Vector& zero_angles)
    {
        set_motor_array(motors, motor_constants, gear_ratios, zero_angles);
    }

    BlmcJointModules()
    {
    }

    void set_motor_array(
            const std::array<std::shared_ptr<blmc_drivers::MotorInterface>,
            COUNT>& motors,
            const Vector& motor_constants,
            const Vector& gear_ratios,
            const Vector& zero_angles)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i] = std::make_shared<BlmcJointModule>(motors[i],
                                                            motor_constants[i],
                                                            gear_ratios[i],
                                                            zero_angles[i]);
        }
    }

    void send_torques()
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->send_torque();
        }
    }

    void set_torques(const Vector& desired_torques)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
    }

    Vector get_sent_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_sent_torque();
        }
        return torques;
    }

    Vector get_measured_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_measured_torque();
        }
        return torques;
    }

    Vector get_measured_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_measured_angle();
        }
        return positions;
    }

    Vector get_measured_velocities() const
    {
        Vector velocities;

        for(size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = modules_[i]->get_measured_velocity();
        }
        return velocities;
    }

    void set_zero_angles(const Vector& zero_angles)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_zero_angle(zero_angles(i));
        }
    }

    Vector get_zero_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_zero_angle();
        }
        return positions;
    }

protected:
    Vector get_index_angles() const
    {
        Vector index_angles;

        for(size_t i = 0; i < COUNT; i++)
        {
            index_angles(i) = modules_[i]->get_index_angle();
        }
        return index_angles;
    }


private:
    std::array<std::shared_ptr<BlmcJointModule>, COUNT> modules_;

};





} // namespace blmc_robots
