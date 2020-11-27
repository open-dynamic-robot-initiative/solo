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
#include "blmc_robots/common_header.hpp"
#include "blmc_robots/mathematics/polynome.hpp"

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <stdexcept>

namespace blmc_robots
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

        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->set_current_sat(max_currents(i));
        }
    }

    /**
     * @brief Enable all motors and motor drivers used by the joint module.
     */
    void enable()
    {

        cpt = 0;
        double dt = 0.001;
        t = 0;
        double kp = 5.;
        double kd = 0.1;
        double iq_sat = 4.0;
        double freq = 0.5;
        double amplitude = M_PI / 10.;
        double init_pos[N_SLAVES * 2] = {0};
        int state = 0;
        int N_SLAVES_CONTROLED = 6;

        nice(-20); //give the process a high priority
        printf("-- Main --\n");
        //assert(argc > 1);
        // robot_if_->Init();
        //Initialisation, send the init commands
        for (int i = 0; i < N_SLAVES_CONTROLED; i++)
        {
            robot_if_->motor_drivers[i].motor1->SetCurrentReference(0);
            robot_if_->motor_drivers[i].motor2->SetCurrentReference(0);
            robot_if_->motor_drivers[i].motor1->Enable();
            robot_if_->motor_drivers[i].motor2->Enable();
            
            // Set the gains for the PD controller running on the cards.
            robot_if_->motor_drivers[i].motor1->set_kp(kp);
            robot_if_->motor_drivers[i].motor2->set_kp(kp);
            robot_if_->motor_drivers[i].motor1->set_kd(kd);
            robot_if_->motor_drivers[i].motor2->set_kd(kd);

            // Set the maximum current controlled by the card.
            robot_if_->motor_drivers[i].motor1->set_current_sat(iq_sat);
            robot_if_->motor_drivers[i].motor2->set_current_sat(iq_sat);
            
            robot_if_->motor_drivers[i].EnablePositionRolloverError();
            robot_if_->motor_drivers[i].SetTimeout(5);
            robot_if_->motor_drivers[i].Enable();
        }

        last = std::chrono::system_clock::now();
        while (!robot_if_->IsTimeout() && !robot_if_->IsAckMsgReceived()) {
            if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
            {
                last = std::chrono::system_clock::now();
                robot_if_->SendInit();
            }
        }

        if (robot_if_->IsTimeout())
        {
            printf("Timeout while waiting for ack.\n");
        }

        int done = 0;
        while (!robot_if_->IsTimeout() and done == 0)
        {
            if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
            {
                last = std::chrono::system_clock::now(); //last+dt would be better
                cpt++;
                t += dt;
                robot_if_->ParseSensorData(); // This will read the last incomming packet and update all sensor fields.
                switch (state)
                {
                case 0: //check the end of calibration (are the all controlled motor enabled and ready?)
                    state = 1;
                    for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                    {
                        if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                        if (!(robot_if_->motors[i].IsEnabled() && robot_if_->motors[i].IsReady()))
                        {
                            state = 0;
                        }
                        init_pos[i] = robot_if_->motors[i].GetPosition(); //initial position

                        // Use the current state as target for the PD controller.
                        robot_if_->motors[i].SetCurrentReference(0.);
                        robot_if_->motors[i].SetPositionReference(init_pos[i]);
                        robot_if_->motors[i].SetVelocityReference(0.);

                        t = 0;  //to start sin at 0
                    }
                    break;
                case 1:
                    done = 1;
                    //closed loop, position
                    for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                    {
                        if (i % 2 == 0)
                        {
                            if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave
     
                            // making sure that the transaction with the corresponding µdriver board succeeded
                            if (robot_if_->motor_drivers[i / 2].error_code == 0xf)
                            {
                                //printf("Transaction with SPI%d failed\n", i / 2);
                                continue; //user should decide what to do in that case, here we ignore that motor
                            }
                        }

                        if (robot_if_->motors[i].IsEnabled())
                        {
                            double ref = init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
                            double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);

                            robot_if_->motors[i].SetCurrentReference(0.);
                            robot_if_->motors[i].SetPositionReference(ref);
                            robot_if_->motors[i].SetVelocityReference(v_ref);
                        }
                    }
                    break;
                }
                if (cpt % 100 == 0)
                {
                    // printf("\33[H\33[2J"); //clear screen
                    // robot_if_->PrintIMU();
                    // robot_if_->PrintADC();
                    // robot_if_->PrintMotors();
                    // robot_if_->PrintMotorDrivers();
                    // robot_if_->PrintStats();
                    // fflush(stdout);
                     

                }
                robot_if_->SendCommand(); //This will send the command packet
            }
            else
            {
                std::this_thread::yield();
            }
        }





        // while (1)
        // {
        //     cpt++;
        //     t += dt;
        //     robot_if_->ParseSensorData(); // This will read the last incomming packet and update all sensor fields.

        //     //closed loop, position
        //     for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
        //     {
        //         if (i % 2 == 0)
        //         {
        //             if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

        //             // making sure that the transaction with the corresponding µdriver board succeeded
        //             if (robot_if_->motor_drivers[i / 2].error_code == 0xf)
        //             {
        //                 //printf("Transaction with SPI%d failed\n", i / 2);
        //                 continue; //user should decide what to do in that case, here we ignore that motor
        //             }
        //         }

        //         if (robot_if_->motors[i].IsEnabled())
        //         {
        //             double ref = init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
        //             double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);

        //             robot_if_->motors[i].SetCurrentReference(0.);
        //             robot_if_->motors[i].SetPositionReference(ref);
        //             robot_if_->motors[i].SetVelocityReference(v_ref);
        //         }
        //     }

        //     if (cpt % 100 == 0)
        //     {
        //         printf("\33[H\33[2J"); //clear screen
        //         robot_if_->PrintIMU();
        //         robot_if_->PrintADC();
        //         robot_if_->PrintMotors();
        //         robot_if_->PrintMotorDrivers();
        //         robot_if_->PrintStats();
        //         fflush(stdout);
                 

        //     }
        //     robot_if_->SendCommand(); //This will send the command packet

        // }

        // while (!robot_if_->IsTimeout())
        // {
        //     if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
        //     {
        //         last = std::chrono::system_clock::now(); //last+dt would be better
        //         cpt++;
        //         t += dt;
        //         robot_if_->ParseSensorData(); // This will read the last incomming packet and update all sensor fields.

        //         //closed loop, position
        //         for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
        //         {
        //             if (i % 2 == 0)
        //             {
        //                 if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave
 
        //                 // making sure that the transaction with the corresponding µdriver board succeeded
        //                 if (robot_if_->motor_drivers[i / 2].error_code == 0xf)
        //                 {
        //                     //printf("Transaction with SPI%d failed\n", i / 2);
        //                     continue; //user should decide what to do in that case, here we ignore that motor
        //                 }
        //             }

        //             if (robot_if_->motors[i].IsEnabled())
        //             {
        //                 double ref = init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
        //                 double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);

        //                 robot_if_->motors[i].SetCurrentReference(0.);
        //                 robot_if_->motors[i].SetPositionReference(ref);
        //                 robot_if_->motors[i].SetVelocityReference(v_ref);
        //             }
        //         }

        //         if (cpt % 100 == 0)
        //         {
        //             printf("\33[H\33[2J"); //clear screen
        //             robot_if_->PrintIMU();
        //             robot_if_->PrintADC();
        //             robot_if_->PrintMotors();
        //             robot_if_->PrintMotorDrivers();
        //             robot_if_->PrintStats();
        //             fflush(stdout);
                     

        //         }
        //         robot_if_->SendCommand(); //This will send the command packet
        //     }
        //     else
        //     {
        //         std::this_thread::yield();
        //     }
        // }


        // printf("\33[H\33[2J"); //clear screen
        // robot_if_->PrintIMU();
        // robot_if_->PrintADC();
        // robot_if_->PrintMotors();
        // robot_if_->PrintMotorDrivers();
        // robot_if_->PrintStats();
        // fflush(stdout);

        // // Establish connection.
        // auto last = std::chrono::system_clock::now();
        // while (!robot_if_->IsTimeout() && !robot_if_->IsAckMsgReceived()) 
        // {
        //     if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
        //     {
        //         last = std::chrono::system_clock::now();
        //         robot_if_->SendInit();
        //     }
        // }

        // // Enable the system.
        // for (int i = 0; i < COUNT; i++)
        // {
        //     int driver_idx = motor_to_card_index_[i];
        //     robot_if_->motor_drivers[driver_idx].motor1->SetCurrentReference(0);
        //     robot_if_->motor_drivers[driver_idx].motor2->SetCurrentReference(0);
        //     robot_if_->motor_drivers[driver_idx].motor1->Enable();
        //     robot_if_->motor_drivers[driver_idx].motor2->Enable();
        //     robot_if_->motor_drivers[driver_idx].EnablePositionRolloverError();
        //     robot_if_->motor_drivers[driver_idx].SetTimeout(5);
        //     robot_if_->motor_drivers[driver_idx].Enable();
        // }

        // robot_if_->SendCommand();
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
    void send_commands()
    {

        double dt = 0.001;
        double kp = 5.;
        double kd = 0.1;
        double iq_sat = 4.0;
        double freq = 0.5;
        double amplitude = M_PI / 10.;
        double init_pos[N_SLAVES * 2] = {0};
        int state = 0;
        int N_SLAVES_CONTROLED = 6;


      
        // if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
        // {
        //     last = std::chrono::system_clock::now(); //last+dt would be better
            cpt++;
            // t += dt;
            // // robot_if_->ParseSensorData(); // This will read the last incomming packet and update all sensor fields.
            
            // //closed loop, position
            // for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
            // {
            //     if (i % 2 == 0)
            //     {
            //         if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

            //         // making sure that the transaction with the corresponding µdriver board succeeded
            //         if (robot_if_->motor_drivers[i / 2].error_code == 0xf)
            //         {
            //             //printf("Transaction with SPI%d failed\n", i / 2);
            //             continue; //user should decide what to do in that case, here we ignore that motor
            //         }
            //     }

            //     if (robot_if_->motors[i].IsEnabled())
            //     {
            //         double ref = init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
            //         double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);

            //         // robot_if_->motors[i].SetCurrentReference(0.);
            //         // robot_if_->motors[i].SetPositionReference(ref);
            //         // robot_if_->motors[i].SetVelocityReference(v_ref);
            //     }
            // }
            if (cpt % 100 == 0)
            {
                // Vector x = polarities_
                //                      .cwiseQuotient(gear_ratios_)
                //                      .cwiseQuotient(motor_constants_);
                // printf("\33[H\33[2J"); //clear screen
                //        // "      0.50 |       0.00 |      -0.00 |       0.40 |       0.01 |     -11.11 | "
                // printf(" position  |  velocity  | ff_current |     kp     |     kd     |current_ref |\n");
                // for (int i = 0; i < 4 * 2; i++)
                // {

                //     printf("%10.02f | ", robot_if_->motors[i].get_position_ref());
                //     printf("%10.02f | ", robot_if_->motors[i].get_velocity_ref());
                //     printf("%10.02f | ", robot_if_->motors[i].get_current_ref());
                //     printf("%10.02f | ", robot_if_->motors[i].get_kp());
                //     printf("%10.02f | ", robot_if_->motors[i].get_kd());
                //     printf("%10.02f | ", robot_if_->motors[i].get_current_sat());
                //     printf("%10.02f | ", motor_constants_[i]);
                //     printf("%10.02f | ", gear_ratios_[i]);
                //     printf("%10.02f | ", polarities_[i]);
                //     printf("%10.02f | ", x[i]);
                //     // printf("%5.2d | ", 2 * i);
                //     // printf("%5.2f | ", robot_if_->motors[i].get_position_ref());
                //     printf("\n");

                // }
                // printf("\n");


                // robot_if_->PrintIMU();
                // robot_if_->PrintADC();
                // robot_if_->PrintMotors();
                // robot_if_->PrintMotorDrivers();
                // robot_if_->PrintStats();
                // fflush(stdout);
                 

            }
            robot_if_->SendCommand(); //This will send the command packet
        // }
        // else
        // {
        //     std::this_thread::yield();
        // }


        // //closed loop, position
        // for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
        // {
        //     if (i % 2 == 0)
        //     {
        //         if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

        //         // making sure that the transaction with the corresponding µdriver board succeeded
        //         if (robot_if_->motor_drivers[i / 2].error_code == 0xf)
        //         {
        //             //printf("Transaction with SPI%d failed\n", i / 2);
        //             continue; //user should decide what to do in that case, here we ignore that motor
        //         }
        //     }

        //     if (robot_if_->motors[i].IsEnabled())
        //     {
        //         double ref = init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
        //         double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);

        //         robot_if_->motors[i].SetCurrentReference(0.);
        //         robot_if_->motors[i].SetPositionReference(ref);
        //         robot_if_->motors[i].SetVelocityReference(v_ref);
        //     }
        // }

        // if (cpt % 100 == 0)
        // {
        //     printf("\33[H\33[2J"); //clear screen
        //     robot_if_->PrintIMU();
        //     robot_if_->PrintADC();
        //     robot_if_->PrintMotors();
        //     robot_if_->PrintMotorDrivers();
        //     robot_if_->PrintStats();
        //     fflush(stdout);
             

        // }
        // robot_if_->SendCommand(); //This will send the command packet

        // int cpt = 0;
        // double dt = 0.001;
        // double t = 0;
        // double kp = 5.;
        // double kd = 0.1;
        // double iq_sat = 4.0;
        // double freq = 0.5;
        // double amplitude = M_PI / 10.;
        // double init_pos[N_SLAVES * 2] = {0};
        // int state = 0;
        // int N_SLAVES_CONTROLED = 6;

        // for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
        // {
        //     if (i % 2 == 0)
        //     {
        //         if (!robot_if_->motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

        //         // making sure that the transaction with the corresponding µdriver board succeeded
        //         if (robot_if_->motor_drivers[i / 2].error_code == 0xf)
        //         {
        //             //printf("Transaction with SPI%d failed\n", i / 2);
        //             continue; //user should decide what to do in that case, here we ignore that motor
        //         }
        //     }

        //     if (robot_if_->motors[i].IsEnabled())
        //     {
        //         double ref = init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
        //         double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);

        //         robot_if_->motors[i].SetCurrentReference(0.);
        //         robot_if_->motors[i].SetPositionReference(ref);
        //         robot_if_->motors[i].SetVelocityReference(v_ref);
        //     }
        // }


        // robot_if_->SendCommand();
        // printf("\33[H\33[2J"); //clear screen
        // robot_if_->PrintIMU();
        // robot_if_->PrintADC();
        // robot_if_->PrintMotors();
        // robot_if_->PrintMotorDrivers();
        // robot_if_->PrintStats();
        // fflush(stdout);
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

    void set_joint_desired_position(const Vector& joint_desired_position)
    {
        Vector motor_desired_position = polarities_.cwiseProduct(joint_desired_position)
                                     .cwiseProduct(gear_ratios_);
        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->SetPositionReference(motor_desired_position(i));
        }
    }

    void set_joint_desired_velocity(const Vector& joint_desired_velocity)
    {
        Vector motor_desired_velocity = polarities_.cwiseProduct(joint_desired_velocity)
                                     .cwiseProduct(gear_ratios_);
        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->SetVelocityReference(motor_desired_velocity(i));
        }
    }

    void set_joint_torque_saturation(const Vector& joint_torque_saturation)
    {
        Vector max_current = joint_torque_saturation
                                     .cwiseQuotient(gear_ratios_)
                                     .cwiseQuotient(motor_constants_);

        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->set_current_sat(max_current(i));
        }
    }

    void set_joint_kp(const Vector& joint_kp)
    {
        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->set_kp(joint_kp(i));
        }
    }

    void set_joint_kd(const Vector& joint_kd)
    {
        for (int i = 0; i < motors_.size(); i++)
        {
            motors_[i]->set_kd(joint_kd(i));
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

    std::chrono::time_point<std::chrono::system_clock> last;
    int cpt;
    double t;
};

}  // namespace blmc_robots
