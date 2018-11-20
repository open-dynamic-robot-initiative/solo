#pragma once

#include <array>

#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <blmc_drivers/devices/motor.hpp>


namespace blmc_robots
{

class BlmcJointModule
{
public:

    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;


    BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
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
            const Vector& zero_positions)
    {
      set_motor_array(motors, motor_constants, gear_ratios, zero_positions);
    }

    BlmcJointModules()
    {
    }

    void set_motor_array(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>& motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_positions)
    {
      for(size_t i = 0; i < COUNT; i++)
      {
          modules_[i] = std::make_shared<BlmcJointModule>(motors[i],
                                                     motor_constants[i],
                                                     gear_ratios[i],
                                                     zero_positions[i]);
      }
    }

    void set_torques(const Vector& desired_torques)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
    }

    void send_torques()
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->send_torque();
        }
    }

    Vector get_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_torque();
        }
        return torques;
    }

    Vector get_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_angle();
        }
        return positions;
    }

    Vector get_angular_velocities() const
    {
        Vector velocities;

        for(size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = modules_[i]->get_angular_velocity();
        }
        return velocities;
    }

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
