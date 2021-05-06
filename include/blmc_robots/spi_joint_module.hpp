/**
 * @file blmc_joint_module.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-02-17
 */
#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <array>
#include <iostream>
#include <stdexcept>

#include "blmc_drivers/devices/motor.hpp"
#include "solo/common_header.hpp"

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <stdexcept>

namespace solo
{
/**
 * @brief This class defines an interface to a collection of BLMC joints. It
 * creates a BLMCJointModule for every blmc_driver::MotorInterface provided.
 */
template <int COUNT>
class SpiJointModules
{
public:
    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface.
     */
    typedef Eigen::Matrix<double, COUNT, 1> Vector;

    /**
     * @brief Construct a new SpiJointModules object.
     */
    SpiJointModules(std::shared_ptr<MasterBoardInterface> robot_if,
                    std::array<int, COUNT>& motor_to_card_index,
                    std::array<int, COUNT>& motor_to_card_port_index,
                    const Vector& motor_constants,
                    const Vector& gear_ratios,
                    const Vector& zero_angles,
                    const Vector& max_currents,
                    std::array<bool, COUNT> reverse_polarities)
    {
        robot_if_ = robot_if;

        // Setup the motor vectores based on the card and port mapping.
        for (int i = 0; i < COUNT; i++)
        {
            int driver_idx = motor_to_card_index[i];
            if (motor_to_card_port_index[i] == 0)
            {
                motors_[i] = (robot_if->motor_drivers[driver_idx].motor1);
            }
            else
            {
                motors_[i] = (robot_if->motor_drivers[driver_idx].motor2);
            }

            polarities_[i] = reverse_polarities[i] ? -1. : 1.;
        }

        motor_constants_ = motor_constants;
        gear_ratios_ = gear_ratios;
        zero_angles_ = zero_angles;
        max_currents_ = max_currents;

        motor_to_card_index_ = motor_to_card_index;

        index_angles_.fill(0.);
    }

    /**
     * @brief Enable all motors and motor drivers used by the joint module.
     */
    void enable()
    {
        for (int i = 0; i < COUNT; i++)
        {
            int driver_idx = motor_to_card_index_[i];
            robot_if_->motor_drivers[driver_idx].motor1->SetCurrentReference(0);
            robot_if_->motor_drivers[driver_idx].motor2->SetCurrentReference(0);
            robot_if_->motor_drivers[driver_idx].motor1->Enable();
            robot_if_->motor_drivers[driver_idx].motor2->Enable();
            robot_if_->motor_drivers[driver_idx].EnablePositionRolloverError();
            robot_if_->motor_drivers[driver_idx].SetTimeout(5);
            robot_if_->motor_drivers[driver_idx].Enable();
        }
        robot_if_->SendCommand();
    }

    /**
     * @brief Checks if all motors report ready.
     *
     * @return True if all motors are ready, false otherwise.
     */
    bool is_ready()
    {
        for (int i = 0; i < COUNT; i++)
        {
            if (!motors_[i]->IsEnabled() || !motors_[i]->IsReady())
            {
                return false;
            }
        }
        return true;
    }

    std::array<bool, COUNT> get_motor_enabled()
    {
        std::array<bool, COUNT> motor_enabled;
        for (int i = 0; i < COUNT; i++)
        {
            motor_enabled[i] = motors_[i]->IsEnabled();
        }
        return motor_enabled;
    }

    std::array<bool, COUNT> get_motor_ready()
    {
        std::array<bool, COUNT> motor_ready;
        for (int i = 0; i < COUNT; i++)
        {
            motor_ready[i] = motors_[i]->IsReady();
        }
        return motor_ready;
    }

    /**
     * @brief Send the registered torques to all modules.
     */
    void send_torques()
    {
        robot_if_->SendCommand();
    }

    /**
     * @brief Updates the measurements based on the lastest package from
     * the master board.
     */
    void acquire_sensors()
    {
        robot_if_->ParseSensorData();

        // Keep tack of the first recorded encoder.
        Vector positions = get_measured_angles();
        for (int i = 0; i < COUNT; i++)
        {
            if (saw_index_[i] == false && motors_[i]->HasIndexBeenDetected())
            {
                saw_index_[i] = true;
                index_angles_[i] = positions(i);
            }
        }
    }

    /**
     * @brief Register the joint torques to be sent for all modules.
     *
     * @param desired_torques (Nm)
     */
    void set_torques(const Vector& desired_torques)
    {
        Vector desired_current = polarities_.cwiseProduct(desired_torques)
                                     .cwiseQuotient(gear_ratios_)
                                     .cwiseQuotient(motor_constants_);

        // Current clamping.
        desired_current = desired_current.cwiseMin(max_currents_);
        desired_current = desired_current.cwiseMax(-max_currents_);
        // Vector desired_current;
        // desired_current.setZero();

        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->SetCurrentReference(desired_current(i));
        }
    }

    /**
     * @brief Get the maximum admissible joint torque that can be applied.
     *
     * @return Vector (N/m)
     */
    Vector get_max_torques()
    {
        return max_currents_.cwiseProduct(gear_ratios_)
            .cwiseProduct(motor_constants_);
    }

    /**
     * @brief Get the previously sent torques.
     *
     * @return Vector (Nm)
     */
    Vector get_sent_torques() const
    {
        Vector torques;
        for (size_t i = 0; i < COUNT; i++)
        {
            torques(i) = motors_[i]->current_ref;
        }
        torques = torques.cwiseProduct(polarities_)
                      .cwiseProduct(gear_ratios_)
                      .cwiseProduct(motor_constants_);
        return torques;
    }

    /**
     * @brief Get the measured joint torques.
     *
     * @return Vector (Nm)
     */
    Vector get_measured_torques() const
    {
        Vector torques;
        for (size_t i = 0; i < COUNT; i++)
        {
            torques(i) = motors_[i]->GetCurrent();
        }
        torques = torques.cwiseProduct(polarities_)
                      .cwiseProduct(gear_ratios_)
                      .cwiseProduct(motor_constants_);
        return torques;
    }

    /**
     * @brief Get the measured joint angles.
     *
     * @return Vector (rad)
     */
    Vector get_measured_angles() const
    {
        Vector positions;
        for (size_t i = 0; i < COUNT; i++)
        {
            positions(i) = motors_[i]->GetPosition();
        }
        positions =
            positions.cwiseProduct(polarities_).cwiseQuotient(gear_ratios_) -
            zero_angles_;
        return positions;
    }

    /**
     * @brief Get the measured joint velocities.
     *
     * @return Vector (rad/s)
     */
    Vector get_measured_velocities() const
    {
        Vector velocities;
        for (size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = motors_[i]->GetVelocity();
        }
        velocities =
            velocities.cwiseProduct(polarities_).cwiseQuotient(gear_ratios_);
        return velocities;
    }

    /**
     * @brief Set the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @param zero_angles (rad)
     */
    void set_zero_angles(const Vector& zero_angles)
    {
        zero_angles_ = zero_angles;
    }
    /**
     * @brief Get the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @return Vector (rad)
     */
    Vector get_zero_angles() const
    {
        return zero_angles_;
    }
    /**
     * @brief Get the index_angles. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     *
     * @return Vector (rad)
     */
    Vector get_measured_index_angles() const
    {
        return index_angles_;
    }

private:
    Vector motor_constants_;
    Vector gear_ratios_;
    Vector max_currents_;
    Vector zero_angles_;
    Vector polarities_;

    std::array<int, COUNT> motor_to_card_index_;

    Vector index_angles_;
    std::array<bool, COUNT> saw_index_;

    /**
     * @brief Holds the motors in the joint order.
     */
    std::array<Motor*, COUNT> motors_;

    std::shared_ptr<MasterBoardInterface> robot_if_;
};

}  // namespace solo
