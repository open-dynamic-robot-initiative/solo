/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the RealFinger
 * \author Manuel Wuthrich
 * \date 2018
 *
 * This file declares the RealFinger class which defines the test
 * bench with 8 motors.
 */

#pragma once


#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <tuple>

#include <blmc_robots/blmc_joint_module.hpp>
#include "real_time_tools/spinner.hpp"

#include <robot_interfaces/finger.hpp>



namespace blmc_robots
{

class RealFinger: public robot_interfaces::Finger
{
public:
    typedef robot_interfaces::Finger::Vector Vector;


    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
    MotorBoards;

    RealFinger(const MotorBoards& motor_boards):
        RealFinger(create_motors(motor_boards))
    {
        motor_boards_ = motor_boards;

        max_torque_ = 2.0 * 0.02 * 9.0;
        double delta_time = 0.01;

        safety_constraints_[base].min_velocity_ = -3.0;
        safety_constraints_[base].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_velocity_ = 3.0;
        safety_constraints_[base].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[base].max_torque_ = max_torque_;
        safety_constraints_[base].inertia_ = 0.003;
        safety_constraints_[base].max_jerk_ =
                2 * max_torque_ / safety_constraints_[base].inertia_ / delta_time;

        safety_constraints_[center].min_velocity_ = -3.0;
        safety_constraints_[center].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[center].max_velocity_ = 3.0;
        safety_constraints_[center].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[center].max_torque_ = max_torque_;
        safety_constraints_[center].inertia_ = 0.0025; // 0.0025 determined experimentally
        safety_constraints_[center].max_jerk_ =
                2 * max_torque_ / safety_constraints_[center].inertia_ / delta_time;

        safety_constraints_[tip].min_velocity_ = -6.0;
        safety_constraints_[tip].min_position_ = std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_velocity_ = 6.0;
        safety_constraints_[tip].max_position_ = -std::numeric_limits<double>::infinity();
        safety_constraints_[tip].max_torque_ = max_torque_;
        safety_constraints_[tip].inertia_ = 0.001;
        safety_constraints_[tip].max_jerk_ =
                2 * max_torque_ / safety_constraints_[tip].inertia_ / delta_time;

        /// \todo: is this the right place to calibrate?

        calibrate();
        pause_motors();

        if(max_angles_[base] < 170 / 180.0 * M_PI ||
                max_angles_[center] < 330 / 180.0 * M_PI ||
                max_angles_[tip] < 320 / 180.0 * M_PI)
        {
            std::cout << "something went wrong with calibration!! angles: "
                      <<  max_angles_.transpose() / M_PI * 180 << std::endl;
            exit(-1);
        }

//        // we want to limit the freedom of this joint
//        max_angles_[center] = 160.0 / 180.0 * M_PI;
//        std::cout << "done calibrating, max_angles: "
//                  << max_angles_.transpose() / M_PI * 180 << std::endl;

//        for(size_t i = 0; i < 3; i++)
//        {
//            safety_constraints_[i].min_position_ = 0.0;
//            safety_constraints_[i].max_position_ = max_angles_[i];
//        }
//        safety_constraints_[center].max_velocity_ = -std::numeric_limits<double>::infinity();


        Vector min_angles(0.06, 0.84, 1.37);
        Vector max_angles(2.8, 1.95, 3.8);

        for(size_t i = 0; i < 3; i++)
        {
            safety_constraints_[i].min_position_ = min_angles[i];
            safety_constraints_[i].max_position_ = max_angles[i];
            safety_constraints_[i].min_velocity_ =
                    std::numeric_limits<double>::infinity();
            safety_constraints_[i].max_velocity_ =
                    -std::numeric_limits<double>::infinity();
        }

    }

private:
    RealFinger(const Motors& motors):
        joint_modules_(motors,
                       0.02 * Vector::Ones(),
                       9.0 * Vector::Ones(),
                       Vector::Zero()) {}

public:
    Vector get_measured_torques() const
    {
        return joint_modules_.get_measured_torques();
    }

    Vector get_measured_angles() const
    {
        return joint_modules_.get_measured_angles();
    }

    Vector get_measured_velocities() const
    {
        return joint_modules_.get_measured_velocities();
    }

    Vector get_tip_pos() const
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
        return Vector::Zero();
    }

    Vector get_object_pos() const
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
        return Vector::Zero();
    }

    void set_object_pos(const Vector& pos)
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
    }

    Quaternion get_object_quat() const
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
        return Quaternion::Zero();
    }

    Vector get_target_pos() const
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
        return Vector::Zero();
    }

    void set_target_pos(const Vector& pos) const
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
    }

    void reset_joints()
    {
        std::cout << "not yet implemented " << std::endl;
        exit(-1);
        // \todo reset finger joints to intial properties
    }

    /// \todo: this should go away. we could handle these issues in the motor_board.
    void pause_motors()
    {
        motor_boards_[0]->pause_motors();
        motor_boards_[1]->pause_motors();
    }

    void wait_for_execution() const
    {
        /// \todo: this needs to be filled in
    }

    unsigned char* render(std::string mode)
    {
        std::cout << "real finger cannot render " << std::endl;
        exit(-1);
    }


    static MotorBoards
    create_motor_boards(const std::string& can_0, const std::string& can_1)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
        can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
        can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

        // set up motor boards -------------------------------------------------
        MotorBoards motor_boards;
        motor_boards[0] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[0]);
        motor_boards[1] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[1]);

        motor_boards[0]->wait_until_ready();
        motor_boards[1]->wait_until_ready();

        return motor_boards;
    }


protected:
    Eigen::Vector3d max_angles_;

    void apply_torques(const Vector& desired_torques)
    {
        /// \todo: the safety checks are now being done in here, but should
        /// come outside
        joint_modules_.set_torques(desired_torques);
        joint_modules_.send_torques();
    }


    static Motors
    create_motors(const MotorBoards& motor_boards)
    {
        // set up motors -------------------------------------------------------
        Motors motors;
        motors[0]  = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1]  = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2]  = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }

    /// \todo: calibrate needs to be cleaned and should probably not be here
    /// there are two identical copies in disentanglement_platform and finger,
    /// which is disgusting.


    /**
     * @brief this is an initial calibration procedure executed before the actual
     * control loop to have angle readings in an absolute coordinate frame.
     *
     * The finger reaches its joint limits by applying a constant negative torque
     * simultaneously in all the joints. Be aware that this procedure assumes safe
     * motors where the maximum velocities are constrained.
     *
     * The calibration procedure is finished once the joint velocities are zero
     * according to a moving average estimation of them.
     */
    void calibrate()
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. without the velocity and
        /// torque limitation in the motor this would be very unsafe
        Vector angle_offsets;
        {
            real_time_tools::Spinner spinner;
            spinner.set_period(0.001);
            std::vector<Vector> running_velocities(1000);
            int running_index = 0;
            Vector sum = Vector::Zero();
            while(running_index < 3000 || (sum.maxCoeff() / 1000.0 > 0.001))
            {
                Vector torques = -1 * get_max_torques();
                constrain_and_apply_torques(torques);
                Vector velocities = get_measured_velocities();
                if (running_index >= 1000)
                    sum = sum - running_velocities[running_index % 1000];
                running_velocities[running_index % 1000] = velocities;
                sum = sum + velocities;
                running_index++;
                spinner.spin();
            }
            int count = 0;
            int linearly_decrease_time_steps = 1000;
            int zero_torque_time_steps = 500;



	    /// \todo: this is yet another hack. we release the tip
	    /// joint before the others such that the tip joint and the center joint
            /// dont fight each other
            count = 0;
            Vector torques;
            while(count < linearly_decrease_time_steps)
            {
                torques = -get_max_torques();

		torques[2] = ((linearly_decrease_time_steps -
                                   count + 0.0) / linearly_decrease_time_steps) * torques[2];
                constrain_and_apply_torques(torques);
                count++;
                spinner.spin();
            }
            count = 0;
            while(count < zero_torque_time_steps)
            {
                constrain_and_apply_torques(torques);
                count++;
                spinner.spin();
            }
            /// ---------------------------------------------------------------------


            count = 0;
            while(count < linearly_decrease_time_steps)
            {
                Vector torques = ((linearly_decrease_time_steps -
                                   count + 0.0) / linearly_decrease_time_steps) *
                        get_max_torques() * -1;
                torques[2] = 0;
                constrain_and_apply_torques(torques);
                count++;
                spinner.spin();
            }
            count = 0;
            while(count < zero_torque_time_steps)
            {
                Vector torques = Vector::Zero();
                constrain_and_apply_torques(torques);
                count++;
                spinner.spin();
            }
            angle_offsets = get_measured_angles();
            joint_modules_.set_zero_angles(angle_offsets);

        }

        {

            Eigen::Vector3d starting_position;
            starting_position << 1.5, 1.5, 0.2;

            double kp = 0.4;
            double kd = 0.0025;
            real_time_tools::Spinner spinner;
            spinner.set_period(0.001);
            int count = 0;
            Eigen::Vector3d last_diff(std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            while (true) {
                Eigen::Vector3d diff = starting_position - get_measured_angles();

                // we implement here a small pd control at the current level
                Eigen::Vector3d desired_torque = kp * diff -
                        kd * get_measured_velocities();

                // Send the current to the motor
                constrain_and_apply_torques(desired_torque);
                spinner.spin();
                if (count % 100 == 0)
                {
                    Eigen::Vector3d diff_difference = last_diff - diff;
                    if (std::abs(diff_difference.norm()) < 1e-5)
                        break;
                    last_diff = diff;
                }
                count++;
            }
            pause_motors();
        }



        {
            real_time_tools::Spinner spinner;
            spinner.set_period(0.001);
            std::vector<Vector> running_velocities(1000);
            int running_index = 0;
            Vector sum = Vector::Zero();
            while(running_index < 3000 || (sum.maxCoeff() / 1000.0 > 0.001))
            {
                Vector torques = get_max_torques();
                constrain_and_apply_torques(torques);
                Vector velocities = get_measured_velocities();
                if (running_index >= 1000)
                    sum = sum - running_velocities[running_index % 1000];
                running_velocities[running_index % 1000] = velocities;
                sum = sum + velocities;
                running_index++;
                spinner.spin();
            }
            int count = 0;
            int linearly_decrease_time_steps = 1000;
            int zero_torque_time_steps = 500;
            while(count < linearly_decrease_time_steps)
            {
                Vector torques = ((linearly_decrease_time_steps -
                                   count + 0.0) / linearly_decrease_time_steps) *
                        get_max_torques();
                constrain_and_apply_torques(torques);
                count++;
                spinner.spin();
            }
            count = 0;
            while(count < zero_torque_time_steps)
            {
                Vector torques = Vector::Zero();
                constrain_and_apply_torques(torques);
                count++;
                spinner.spin();
            }
            max_angles_ = get_measured_angles();
        }


        {

            Eigen::Vector3d starting_position;
            starting_position << 1.5, 1.5, 3.0;

            double kp = 0.4;
            double kd = 0.0025;
            real_time_tools::Spinner spinner;
            spinner.set_period(0.001);
            int count = 0;
            Eigen::Vector3d last_diff(std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            while (true) {
                Eigen::Vector3d diff = starting_position - get_measured_angles();

                // we implement here a small pd control at the current level
                Eigen::Vector3d desired_torque = kp * diff -
                        kd * get_measured_velocities();

                // Send the current to the motor
                constrain_and_apply_torques(desired_torque);
                spinner.spin();
                if (count % 100 == 0)
                {
                    Eigen::Vector3d diff_difference = last_diff - diff;
                    if (std::abs(diff_difference.norm()) < 1e-5)
                        break;
                    last_diff = diff;
                }
                count++;
            }
            pause_motors();
        }
    }

private:
    BlmcJointModules<3> joint_modules_;
    MotorBoards motor_boards_;

};

} // namespace blmc_robots
