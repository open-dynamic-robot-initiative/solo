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
               const double& max_current,
               const double& max_velocity,
               const double& min_joint_angle,
               const double& max_joint_angle,
               const double& motor_constant,
               const double& gear_ratio,
               const double& zero_position)
    {
        motor_ = motor;
        motor_constant_ = motor_constant;
        gear_ratio_ = gear_ratio;
        set_max_current(max_current);
        set_max_velocity(max_velocity);
        set_joint_limits(min_joint_angle, max_joint_angle);
        set_zero_angle(zero_position);
    }

    void set_max_current(const double& max_current)
    {
        assert(max_current > 0);
        max_current_ = max_current;
    }

    void set_max_velocity(const double& max_velocity)
    {
        assert(std::isnan(max_velocity) || max_velocity > 0);
        max_velocity_ = max_velocity;
    }

    void set_joint_limits(const double& min_joint_angle,
            const double& max_joint_angle)
    {
        assert(std::isnan(min_joint_angle) || std::isnan(max_joint_angle) || (
                    min_joint_angle < max_joint_angle &&
                    max_joint_angle - min_joint_angle < 2*M_PI));
        min_joint_angle_ = min_joint_angle;
        max_joint_angle_ = max_joint_angle;
    }

    void set_torque(const double& desired_torque)
    {
        double current_target = desired_torque / gear_ratio_ / motor_constant_;

        // Current safety feature to avoid overheating.
        current_target = std::min(current_target, max_current_);
        current_target = std::max(current_target, -max_current_);

        // Velocity safety feature.
        if (!std::isnan(max_velocity_) && std::fabs(
                get_angular_velocity()) > max_velocity_)
            current_target = 0;

        // Joint limits safety feature.
        if (!std::isnan(max_joint_angle_) && get_angle() > max_joint_angle_)
            current_target = -max_current_;
        if (!std::isnan(min_joint_angle_) && get_angle() < min_joint_angle_)
            current_target = max_current_;

        motor_->set_current_target(current_target);
    }

    void set_zero_angle(const double& zero_position)
    {
        assert(-M_PI < zero_position && zero_position < M_PI);
        zero_angle_ = zero_position;
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
        return measurement_history->newest_element()
                * gear_ratio_ * motor_constant_;
    }

    double get_measured_torque() const
    {
        return get_motor_measurement(mi::current)
                * gear_ratio_ * motor_constant_;
    }

    double get_angle() const
    {
        return get_motor_measurement(mi::position) / gear_ratio_ - zero_angle_;
    }

    double get_angular_velocity() const
    {
        return get_motor_measurement(mi::velocity) / gear_ratio_;

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

    double max_current_;
    double max_velocity_;
    double min_joint_angle_;
    double max_joint_angle_;
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
            const Vector& max_current,
            const Vector& max_velocity,
            const Vector& min_joint_angle,
            const Vector& max_joint_angle,
            const Vector& motor_constants,
            const Vector& gear_ratios,
            const Vector& zero_positions)
    {
      set_motor_array(motors, max_current, max_velocity, min_joint_angle,
              max_joint_angle, motor_constants, gear_ratios, zero_positions);
    }

    BlmcJointModules()
    {
    }

    void set_motor_array(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>,
                COUNT>& motors,
        const Vector& max_current,
        const Vector& max_velocity,
        const Vector& min_joint_angle,
        const Vector& max_joint_angle,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_positions)
    {
      for(size_t i = 0; i < COUNT; i++)
      {
          modules_[i] = std::make_shared<BlmcJointModule>(motors[i],
                  max_current[i],
                  max_velocity[i],
                  min_joint_angle[i],
                  max_joint_angle[i],
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

protected:
    void set_zero_angles(const Vector& zero_positions)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_zero_angle(zero_positions(i));
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
