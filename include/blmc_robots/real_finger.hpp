/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the RealFinger
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 *
 * This file declares the RealFinger class which defines the test
 * bench with 8 motors.
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <tuple>

#include <blmc_robots/blmc_joint_module.hpp>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

#include <robot_interfaces/finger.hpp>

namespace blmc_robots
{
class RealFinger
    : public robot_interfaces::Robot<robot_interfaces::finger::Action,
                                     robot_interfaces::finger::Observation>
{
public:
    typedef robot_interfaces::finger::Action Action;
    typedef robot_interfaces::finger::Observation Observation;
    typedef robot_interfaces::finger::Vector Vector;
    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
        MotorBoards;


    RealFinger(const std::string &can_0, const std::string &can_1)
        : RealFinger(create_motor_boards(can_0, can_1))
    {
    }

    RealFinger(const MotorBoards &motor_boards)
        : RealFinger(create_motors(motor_boards))
    {
        motor_boards_ = motor_boards;

        max_torque_ = 2.0 * 0.02 * 9.0;

        calibrate();

        for (size_t i = 0; i < motor_boards_.size(); i++)
        {
            motor_boards_[i]->pause_motors();
        }
    }

    static MotorBoards create_motor_boards(const std::string &can_0,
                                           const std::string &can_1)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
        can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
        can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

        // set up motor boards -------------------------------------------------
        MotorBoards motor_boards;
        for (size_t i = 0; i < 2; i++)
        {
            motor_boards[i] = std::make_shared<blmc_drivers::CanBusMotorBoard>(
                can_buses[i], 1000,
                10);  /// \TODO: reduce the timeout further!!
        }
        for (size_t i = 0; i < 2; i++)
        {
            motor_boards[i]->wait_until_ready();
        }

        return motor_boards;
    }

    Vector get_measured_index_angles() const
    {
        return joint_modules_.get_measured_index_angles();
    }

protected:
    RealFinger(const Motors &motors)
        : Robot(0.003, 0.005),
          joint_modules_(motors, 0.02 * Vector::Ones(), 9.0 * Vector::Ones(),
                         Vector::Zero())
    {
    }

protected:
    Action apply_action(const Action &desired_action) override
    {
        double start_time_sec = real_time_tools::Timer::get_current_time_sec();

        Observation observation = get_latest_observation();
        Vector kd(0.08, 0.08, 0.04);
        double max_torque = 0.36;
        Action applied_action =
            mct::clamp(desired_action, -max_torque, max_torque);
        applied_action = applied_action - kd.cwiseProduct(observation.velocity);
        applied_action = mct::clamp(applied_action, -max_torque, max_torque);

        joint_modules_.set_torques(applied_action);
        joint_modules_.send_torques();
        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

        return applied_action;
    }

public:
    Observation get_latest_observation() override
    {
        Observation observation;
        observation.angle = joint_modules_.get_measured_angles();
        observation.velocity = joint_modules_.get_measured_velocities();
        observation.torque = joint_modules_.get_measured_torques();
        return observation;
    }

protected:
    void shutdown() override
    {
        for (size_t i = 0; i < motor_boards_.size(); i++)
        {
            motor_boards_[i]->pause_motors();
        }
    }

protected:
    Eigen::Vector3d max_angles_;

    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors -------------------------------------------------------
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }

    /// \todo: calibrate needs to be cleaned and should probably not be here
    /// there are two identical copies in disentanglement_platform and finger,
    /// which is disgusting.

    /**
     * @brief Homing on negative end stop and encoder index.
     *
     * Procedure for finding an absolute zero position (or "home" position) when
     * using relative encoders.
     *
     * All joints first move in positive direction until the index of each
     * encoder is found.  Then all joints move in negative direction until they
     * hit the end stop.  Home position is set to the positions of encoder
     * indices closest to the end stop.
     *
     * By default, the zero position is the same as the home position.  The
     * optional argument home_offset_rad provides a means to move the zero
     * position
     * relative to the home position.  The zero position is computed as
     *
     *     zero position = encoder index position + home offset
     *
     * Movement is done by simply applying a constant torque to the joints.  The
     * amount of torque is a ratio of the configured maximum torque defined by
     * `torque_ratio`.
     *
     * @param torque_ratio Ratio of max. torque that is used to move the joints.
     * @param home_offset_rad Offset between the home position and the desired
     * zero
     *                    position.
     */
    bool home_on_index_after_negative_end_stop(
        double torque_ratio, Vector home_offset_rad = Vector::Zero())
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. Without the velocity and
        /// torque limitation in the motor this would be very unsafe

        //! Min. number of steps when moving to the end stop.
        constexpr uint32_t MIN_STEPS_MOVE_TO_END_STOP = 1000;
        //! Size of the window when computing average velocity.
        constexpr uint32_t SIZE_VELOCITY_WINDOW = 100;
        //! Velocity limit at which the joints are considered to be stopped.
        constexpr double STOP_VELOCITY = 0.001;
        //! Distance after which encoder index search is aborted.
        constexpr double SEARCH_DISTANCE_LIMIT_RAD = 2.0;

        static_assert(MIN_STEPS_MOVE_TO_END_STOP > SIZE_VELOCITY_WINDOW,
                      "MIN_STEPS_MOVE_TO_END_STOP has to be bigger than"
                      " SIZE_VELOCITY_WINDOW to ensure correct computation"
                      " of average velocity.");

        // Move until velocity drops to almost zero (= joints hit the end stops)
        // but at least for MIN_STEPS_MOVE_TO_END_STOP time steps.
        // TODO: add timeout to this loop?
        std::vector<Vector> running_velocities(SIZE_VELOCITY_WINDOW);
        Vector summed_velocities = Vector::Zero();
        int step_count = 0;
        while (step_count < MIN_STEPS_MOVE_TO_END_STOP ||
               (summed_velocities.maxCoeff() / SIZE_VELOCITY_WINDOW >
                STOP_VELOCITY))
        {
            Vector torques = -1 * torque_ratio * get_max_torques();
            apply_action(torques);
            Vector abs_velocities =
                get_latest_observation().velocity.cwiseAbs();

            uint32_t running_index = step_count % SIZE_VELOCITY_WINDOW;
            if (step_count >= SIZE_VELOCITY_WINDOW)
            {
                summed_velocities -= running_velocities[running_index];
            }
            running_velocities[running_index] = abs_velocities;
            summed_velocities += abs_velocities;
            step_count++;
        }

        // need to "pause" as the desired actions queue is not filled while
        // homing is running.

        // Home on encoder index
        HomingReturnCode homing_status = joint_modules_.execute_homing(
            SEARCH_DISTANCE_LIMIT_RAD, home_offset_rad);

        rt_printf("Finished homing.\n");

        return homing_status == HomingReturnCode::SUCCEEDED;
    }

    /**
     * @brief Move to given goal position using PD control.
     *
     * @param goal_pos Angular goal position for each joint.
     * @param kp Gain K_p for the PD controller.
     * @param kd Gain K_d for the PD controller.
     * @param tolerance Allowed position error for reaching the goal.  This is
     *     checked per joint, that is the maximal possible error is +/-tolerance
     *     on each joint.
     * @param timeout_cycles Timeout.  If exceeded before goal is reached, the
     *     procedure is aborted. Unit: Number of control loop cycles.
     * @return True if goal position is reached, false if timeout is exceeded.
     */
    bool move_to_position(const Vector &goal_pos, const double kp,
                          const double kd, const double tolerance,
                          const uint32_t timeout_cycles)
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. Without the velocity and
        /// torque limitation in the motor this would be very unsafe

        bool reached_goal = false;
        int cycle_count = 0;
        Vector desired_torque = Vector::Zero();
        Eigen::Vector3d last_diff(std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max());

        while (!reached_goal && cycle_count < timeout_cycles)
        {
            apply_action(desired_torque);

            const Vector position_error =
                goal_pos - get_latest_observation().angle;
            const Vector velocity = get_latest_observation().velocity;

            // we implement here a small PD control at the current level
            desired_torque = kp * position_error - kd * velocity;

#ifdef VERBOSE
            if (cycle_count % 100 == 0)
            {
                rt_printf(
                    "--\n"
                    "position error: %10.5f, %10.5f, %10.5f\n"
                    "velocity:       %10.5f, %10.5f, %10.5f\n",
                    position_error[0], position_error[1], position_error[2],
                    velocity[0], velocity[1], velocity[2]);
            }
#endif

            // Check if the goal is reached (position error below tolerance and
            // velocity close to zero).
            constexpr double ZERO_VELOCITY = 1e-4;
            reached_goal = ((position_error.array().abs() < tolerance).all() &&
                            (velocity.array().abs() < ZERO_VELOCITY).all());

            cycle_count++;
        }

        return reached_goal;
    }

    /**
     * @brief this is an initial calibration procedure executed before the
     * actual control loop to have angle readings in an absolute coordinate
     * frame.
     *
     * The finger reaches its joint limits by applying a constant negative
     * torque simultaneously in all the joints. Be aware that this procedure
     * assumes safe motors where the maximum velocities are constrained.
     *
     * The calibration procedure is finished once the joint velocities are zero
     * according to a moving average estimation of them.
     */
    void calibrate()
    {
        constexpr double TORQUE_RATIO = 0.6;
        constexpr double CONTROL_GAIN_KP = 0.4;
        constexpr double CONTROL_GAIN_KD = 0.0025;
        constexpr double POSITION_TOLERANCE_RAD = 0.2;
        constexpr double MOVE_TIMEOUT = 2000;

        // Offset between home position and zero.  Defined such that the zero
        // position is at the negative end stop (for compatibility with old
        // homing method).
        Vector home_offset_rad;
        home_offset_rad << -0.54, -0.17, 0.0;

        // Start position to which the robot moves after homing.
        Vector starting_position_rad;
        starting_position_rad << 1.5, 1.5, 3.0;

        joint_modules_.set_position_control_gains(CONTROL_GAIN_KP,
                                                  CONTROL_GAIN_KD);

        bool is_homed = home_on_index_after_negative_end_stop(TORQUE_RATIO,
                                                              home_offset_rad);

        if (is_homed)
        {
            bool reached_goal = move_to_position(
                starting_position_rad, CONTROL_GAIN_KP, CONTROL_GAIN_KD,
                POSITION_TOLERANCE_RAD, MOVE_TIMEOUT);
            if (!reached_goal)
            {
                rt_printf("Failed to reach goal, timeout exceeded.\n");
            }
        }
    }

private:
    /// todo: this should probably go away
    double max_torque_;

    BlmcJointModules<3> joint_modules_;
    MotorBoards motor_boards_;

    /// todo: this should probably go away

public:
    Vector get_max_torques() const { return max_torque_ * Vector::Ones(); }
};

robot_interfaces::finger::ServerPtr create_real_finger_server(
    const std::string &can_0, const std::string &can_1,
    robot_interfaces::finger::DataPtr robot_data)
{
    std::shared_ptr<
        robot_interfaces::Robot<robot_interfaces::finger::Action,
                                robot_interfaces::finger::Observation>>
        robot = std::make_shared<RealFinger>(can_0, can_1);

    auto server = std::make_shared<robot_interfaces::finger::Server>(
        robot, robot_data);
    server->set_max_action_repetitions(-1);

    return server;
}


}  // namespace blmc_robots
