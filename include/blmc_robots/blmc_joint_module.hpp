///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2017-2019, New York University and Max Planck Gesellshaft
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <iostream>
#include <array>
#include <stdexcept>
#include <math.h>
#include <Eigen/Eigen>

#include "blmc_drivers/devices/motor.hpp"
#include "blmc_robots/common_header.hpp"

namespace blmc_robots
{

/**
 * @brief The BlmcJointModule class is containing the joint information. It is
 * here to help converting the data from the motor side to the joint side. It
 * also allows the calibration of the joint position during initialization.
 */
class BlmcJointModule
{
public:

    /**
     * @brief Construct a new BlmcJointModule object
     * 
     * @param motor is the C++ object allowing us to send commands and receive
     * sensor data.
     * @param motor_constant (\f$ k \f$) is the torque constant of the motor 
     * \f$ \tau_{motor} = k * i_{motor} \f$
     * @param gear_ratio is the gear ratio between the motor and the joint.
     * @param zero_angle is the angle between the closest positive motor index
     * and the zero configuration.
     */
    BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle);

    /**
     * @brief Set the joint torque to be sent.
     * 
     * @param desired_torque 
     */
    void set_torque(const double& desired_torque);

    /**
     * @brief Set the zero_angle. The zero_angle is the angle between the
     * closest positive motor index and the zero configuration.
     * 
     * @param zero_angle 
     */
    void set_zero_angle(const double& zero_angle);

    /**
     * @brief send the joint torque to the motor. The conversion between joint
     * torque and motor current is done automatically.
     */
    void send_torque();

    /**
     * @brief Get the sent joint torque.
     * 
     * @return double 
     */
    double get_sent_torque() const;

    /**
     * @brief Get the measured joint torque.
     * 
     * @return double 
     */
    double get_measured_torque() const;

    /**
     * @brief Get the measured angle of the joint.
     * 
     * @return double 
     */
    double get_measured_angle() const;

    /**
     * @brief Get the measured velocity of the joint. This data is computed on
     * board of the control card.
     * 
     * @return double 
     */
    double get_measured_velocity() const;

    /**
     * @brief Get the index_angle_. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     * 
     * @return double 
     */
    double get_index_angle() const;

    /**
     * @brief Get the zero_angle_. These are the angle between the starting pose
     * and the theoretical zero pose.
     * 
     * @return double 
     */
    double get_zero_angle() const;

private:
    /**
     * @brief Convert from joint torque to motor current.
     * 
     * @param[in] torque is the input joint
     * @return double the equivalent motor current.
     */
    double joint_torque_to_motor_current(double torque) const;

    /**
     * @brief Convert from motor current to joint torque.
     * 
     * @param current is the motor current.
     * @return double is the equivalent joint torque.
     */
    double motor_current_to_joint_torque(double current) const;

    /**
     * @brief Get motor measurements and check if there are data or not.
     * 
     * @param measurement_index is the id of the measurement you want to get.
     * check: blmc_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    double get_motor_measurement(const mi& measurement_index) const;

    /**
     * @brief This is the pointer to the motor interface.
     */
    std::shared_ptr<blmc_drivers::MotorInterface> motor_;

    /**
     * @brief This is the torque constant of the motor:
     * \f$ \tau_{motor} = k * i_{motor} \f$
     */
    double motor_constant_;
    /**
     * @brief This correspond to the reduction (\f$ \beta \f$) between the motor rotation and
     * the joint. \f$ \theta_{joint} = \theta_{motor} / \beta \f$
     */
    double gear_ratio_;
    /**
     * @brief This is the distance between the closest positive index and the
     * zero configuration.
     */
    double zero_angle_;
};



/**
 * @brief This class defines an interface to a collection of BLMC joints. It
 * creates a BLMCJointModule for every blmc_driver::MotorInterface provided.
 * 
 * @tparam COUNT 
 */
template <int COUNT>
class BlmcJointModules
{
public:
    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface.
     */
    typedef Eigen::Matrix<double, COUNT, 1> Vector;

    /**
     * @brief Construct a new BlmcJointModules object
     * 
     * @param motors 
     * @param motor_constants 
     * @param gear_ratios 
     * @param zero_angles 
     */
    BlmcJointModules(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
          motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles)
    {
        set_motor_array(motors, motor_constants, gear_ratios, zero_angles);
    }
    /**
     * @brief Construct a new BlmcJointModules object
     */
    BlmcJointModules()
    {
    }
    /**
     * @brief Set the motor array, by creating the corresponding modules.
     * 
     * @param motors 
     * @param motor_constants 
     * @param gear_ratios 
     * @param zero_angles 
     */
    void set_motor_array(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
          motors,
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
    /**
     * @brief Send the registered torques to all modules.
     */
    void send_torques()
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->send_torque();
        }
    }

    /**
     * @brief Register the joint torques to be sent for all modules.
     * 
     * @param desired_torques 
     */
    void set_torques(const Vector& desired_torques)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
    }

    /**
     * @brief Get the previously sent torques.
     * 
     * @return Vector 
     */
    Vector get_sent_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_sent_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint torques.
     * 
     * @return Vector 
     */
    Vector get_measured_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_measured_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint angles.
     * 
     * @return Vector 
     */
    Vector get_measured_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_measured_angle();
        }
        return positions;
    }
    /**
     * @brief Get the measured joint velocities.
     * 
     * @return Vector 
     */
    Vector get_measured_velocities() const
    {
        Vector velocities;

        for(size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = modules_[i]->get_measured_velocity();
        }
        return velocities;
    }

    /**
     * @brief Set the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     * 
     * @param zero_angles 
     */
    void set_zero_angles(const Vector& zero_angles)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_zero_angle(zero_angles(i));
        }
    }
    /**
     * @brief Get the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     * 
     * @return Vector 
     */
    Vector get_zero_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_zero_angle();
        }
        return positions;
    }
    /**
     * @brief Get the index_angles. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     * 
     * @return Vector 
     */
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
    /**
     * @brief These are the BLMCJointModule objects corresponding to a robot.
     */
    std::array<std::shared_ptr<BlmcJointModule>, COUNT> modules_;

};





} // namespace blmc_robots
