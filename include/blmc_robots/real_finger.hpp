/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the RealFinger
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck
 Gesellshaft.

 * This file declares the RealFinger class which defines the test
 * bench with 8 motors.
 */

#pragma once

#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <tuple>

#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"
#include <blmc_robots/blmc_joint_module.hpp>

#include <robot_interfaces/finger.hpp>

namespace blmc_robots {

class RealFinger : public robot_interfaces::Finger {
public:
  typedef robot_interfaces::Finger::Vector Vector;
  typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> Motors;
  typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
      MotorBoards;

  static std::shared_ptr<Finger> create(const std::string &can_0,
                                        const std::string &can_1) {
    auto finger = std::make_shared<RealFinger>(can_0, can_1);
    return finger;
  }

  RealFinger(const std::string &can_0, const std::string &can_1)
      : RealFinger(create_motor_boards(can_0, can_1)) {}

  RealFinger(const MotorBoards &motor_boards)
      : RealFinger(create_motors(motor_boards)) {
    motor_boards_ = motor_boards;
    pause();

    max_torque_ = 2.0 * 0.02 * 9.0;

    calibrate();
    pause();
  }

    static MotorBoards create_motor_boards(const std::string &can_0,
                                         const std::string &can_1) {
    // setup can buses -----------------------------------------------------
    std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
    can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
    can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

    // set up motor boards -------------------------------------------------
    MotorBoards motor_boards;
    for (size_t i = 0; i < 2; i++) {
      motor_boards[i] = std::make_shared<blmc_drivers::CanBusMotorBoard>(
          can_buses[i], 1000,
          10); /// \TODO: reduce the timeout further!!
    }
    for (size_t i = 0; i < 2; i++) {
      motor_boards[i]->wait_until_ready();
    }

    return motor_boards;
  }

  Vector get_measured_index_angles() const {
    return joint_modules_.get_measured_index_angles();
  }

protected:
  RealFinger(const Motors &motors)
      : joint_modules_(motors, 0.02 * Vector::Ones(), 9.0 * Vector::Ones(),
                       Vector::Zero()) {}

  virtual Observation get_latest_observation() {
    Observation observation;
    observation.angle = joint_modules_.get_measured_angles();
    observation.velocity = joint_modules_.get_measured_velocities();
    observation.torque = joint_modules_.get_measured_torques();
    return observation;
  }


protected:
  Eigen::Vector3d max_angles_;

  void apply_torques(const Vector &desired_torques) {
    /// \todo: the safety checks are now being done in here, but should
    /// come outside
    joint_modules_.set_torques(desired_torques);
    joint_modules_.send_torques();
  }

  virtual void apply_action(const Action &action) {
    double start_time_sec = real_time_tools::Timer::get_current_time_sec();
    joint_modules_.set_torques(action);
    joint_modules_.send_torques();
    real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);
  }

  static Motors create_motors(const MotorBoards &motor_boards) {
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
     * Movement is done by simply applying a constant torque to the joints.  The
     * amount of torque is a ratio of the configured maximum torque defined by
     * `torque_ratio`.
     *
     * @param torque_ratio Ratio of max. torque that is used to move the joints.
     */
    void home_on_index_after_negative_end_stop(double torque_ratio)
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. Without the velocity and
        /// torque limitation in the motor this would be very unsafe

        //! Max. number of steps when searching for the encoder indices.
        const uint32_t MAX_STEPS_SEARCH_INDEX = 1000;
        //! Min. number of steps when moving to the end stop.
        const uint32_t MIN_STEPS_MOVE_TO_END_STOP = 1000;
        //! Size of the window when computing average velocity.
        const uint32_t SIZE_VELOCITY_WINDOW = 100;
        //! Velocity limit at which the joints are considered to be stopped.
        const double STOP_VELOCITY = 0.001;


        // First move a bit in positive direction until encoder indices are
        // found or timeout triggers.
        int count = 0;
        while (joint_modules_.get_measured_index_angles().hasNaN()
                && count < MAX_STEPS_SEARCH_INDEX) {
            Vector torques = torque_ratio * get_max_torques();
            TimeIndex t = append_desired_action(torques);
            wait_until_time_index(t);
            count++;
        }
        // Note: It might be possible that one or more encoder indices are not
        // found before the timeout triggers (e.g. when joint is already at the
        // positive end stop).  Still continue, as in this case the index might
        // be found while moving to the negative end stop in the following step.
        // If index is still not found for some reason, this is caught by a
        // check at the end.


        // Move until velocity drops to almost zero (= joints hit the end stops)
        // but at least for MIN_STEPS_MOVE_TO_END_STOP time steps.
        // TODO: add timeout to this loop?
        std::vector<Vector> running_velocities(SIZE_VELOCITY_WINDOW);
        Vector summed_velocities = Vector::Zero();
        count = 0;
        while (count < MIN_STEPS_MOVE_TO_END_STOP
                || (summed_velocities.maxCoeff() / SIZE_VELOCITY_WINDOW > STOP_VELOCITY))
        {
            Vector torques = -1 * torque_ratio * get_max_torques();
            TimeIndex t = append_desired_action(torques);
            Vector abs_velocities = get_observation(t).velocity.cwiseAbs();

            uint32_t running_index = count % SIZE_VELOCITY_WINDOW;
            if (count >= SIZE_VELOCITY_WINDOW) {
                summed_velocities -= running_velocities[running_index];
            }
            running_velocities[running_index] = abs_velocities;
            summed_velocities += abs_velocities;
            count++;
        }


        // At this point we assume that all encoder indices were already
        // encountered, so the last one seen is the one closest to the end stop
        // --> no need to move to the index, simply set home position to the
        // position where the index was last seen.
        Vector encoder_index_angles = joint_modules_.get_measured_index_angles();
        if (encoder_index_angles.hasNaN()) {
            // Make sure we do not try to home on the index when the index
            // was not found.  This should normally not happen, so no specific
            // error handling is done here.
            rt_printf("Homing failed, index not found.  Exit.\n");
            exit(-1);
        }

        joint_modules_.set_zero_angles(encoder_index_angles);

        rt_printf("Home position: %f, %f, %f\n", encoder_index_angles[0],
                  encoder_index_angles[1], encoder_index_angles[2]);
    }


    /**
     * @brief Move to given goal position using PD control.
     *
     * @param goal_pos Angular goal position for each joint.
     * @param kp Gain K_p for the PD controller.
     * @param kd Gain K_d for the PD controller.
     */
    void move_to_position(Vector goal_pos, double kp, double kd)
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. Without the velocity and
        /// torque limitation in the motor this would be very unsafe

        bool reached_goal = false;
        int count = 0;
        Eigen::Vector3d last_diff(std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        while (!reached_goal && count < 2000)
        {
            Eigen::Vector3d diff = goal_pos - get_observation(current_time_index()).angle;

            // we implement here a small pd control at the current level
            Eigen::Vector3d desired_torque = kp * diff -
                                             kd * joint_modules_.get_measured_velocities();

            // Send the current to the motor
            TimeIndex t = append_desired_action(desired_torque);
            wait_until_time_index(t);
            if (count % 100 == 0)
            {
                Eigen::Vector3d diff_difference = last_diff - diff;
                if (std::abs(diff_difference.norm()) < 1e-5) {
                    reached_goal = true;
                }
                last_diff = diff;
            }
            count++;
        }
    }

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

        const double TORQUE_RATIO = 0.6;

        home_on_index_after_negative_end_stop(TORQUE_RATIO);

        Eigen::Vector3d starting_position;
        starting_position << 1.33, 0.96, 3.0;
        move_to_position(starting_position, 0.4, 0.0025);

        pause();
    }

private:
  BlmcJointModules<3> joint_modules_;
  MotorBoards motor_boards_;
};

} // namespace blmc_robots
