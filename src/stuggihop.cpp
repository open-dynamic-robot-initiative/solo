#include "blmc_robots/stuggihop.hpp"
#include <math.h>

namespace blmc_robots
{
Stuggihop::Stuggihop()
{
    /**
     * Motor data
     */
    motor_positions_.setZero();
    motor_velocities_.setZero();
    motor_currents_.setZero();
    motor_torques_.setZero();
    motor_inertias_.setZero();
    motor_encoder_indexes_.setZero();
    motor_target_currents_.setZero();
    motor_target_torques_.setZero();
    motor_torque_constants_.setZero();
    target_motor_current_tmp_.setZero();

    /**
     * Joint data
     */
    joint_positions_.setZero();
    joint_velocities_.setZero();
    joint_torques_.setZero();
    joint_target_torques_.setZero();
    joint_gear_ratios_.setZero();
    joint_hardstop2zero_offsets_.setZero();
    joint_start2hardstop_offsets_.setZero();

    /**
     * Additional data
     */
    // slider_positions_.setZero();
    motor_max_current_.setZero();

    /**
     * Boom parameters
     */
    boom_gear_ratios_ << 2.88, 1;  // horizontal and vertical
    boom_encoder_index_.setZero();
    boom_zero_positions_.setZero();
    boom_zero_positions_.setZero();
    boom_length_ << 2;  // meters
    base_positions_.setZero();
    base_velocities_.setZero();

    /**
     * Setup some known data
     */

    // for now this value is very small but it is currently for debug mode
    motor_max_current_.fill(4.5);
    motor_torque_constants_.fill(0.025);
    motor_inertias_.fill(0.045);
    joint_gear_ratios_.fill(9.0);
    joint_max_torque_ = motor_max_current_.array() *
                        motor_torque_constants_.array() *
                        joint_gear_ratios_.array();
}

void Stuggihop::initialize()
{
    // initialize the communication with the can cards
    // for(unsigned i=0 ; i< can_buses_.size() ; ++i)
    for (unsigned i = 0; i < 2; ++i)
    {
        std::ostringstream oss;
        oss << "can" << i;  // hard-coding to "can0" and "can1"
        can_buses_[i] = std::make_shared<blmc_drivers::CanBus>(oss.str());
        can_motor_boards_[i] =
            std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_[i]);
    }

    // sliders_[0] =
    //     std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[0],
    //     1);
    // contact_sensors_[0] =
    //     std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[1],
    //     0);
    // height_sensors_[0] =
    //     std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[1],
    //     1);

    // can 0
    // MOTOR_HFE
    motors_[0] = std::make_shared<blmc_drivers::SafeMotor>(
        can_motor_boards_[0], 1, motor_max_current_[0]);
    // MOTOR_KFE
    motors_[1] = std::make_shared<blmc_drivers::SafeMotor>(
        can_motor_boards_[0], 0, motor_max_current_[1]);

    // can1
    // Boom encoders: horizontal and vertical
    //
    boom_encoders_[0] = std::make_shared<blmc_drivers::SafeMotor>(
        can_motor_boards_[1], 0, motor_max_current_[0]);
    // MOTOR_KFE
    boom_encoders_[1] = std::make_shared<blmc_drivers::SafeMotor>(
        can_motor_boards_[1], 1, motor_max_current_[1]);

    // Call the method to sync the max current with the max torques.
    set_max_current(motor_max_current_);

    // Wait to make sure there is a first package when acquire_sensors() later.
    real_time_tools::Timer::sleep_sec(0.5);
}

void Stuggihop::acquire_sensors()
{
    /**
     * Motor data
     */
    for (unsigned i = 0; i < motors_.size(); ++i)
    {
        // acquire the motors positions
        motor_positions_(i) =
            (motors_[i]->get_measurement(mi::position)->newest_element() +
             (joint_start2hardstop_offsets_(i) -
              joint_hardstop2zero_offsets_(i)) *
                 joint_gear_ratios_(i));
        // acquire the motors velocities
        motor_velocities_(i) =
            motors_[i]->get_measurement(mi::velocity)->newest_element();
        // acquire the motors current
        motor_currents_(i) =
            motors_[i]->get_measurement(mi::current)->newest_element();
        // acquire the last sent current sent
        motor_target_currents_(i) =
            motors_[i]->get_sent_current_target()->newest_element();
        // acquire the encoder indexes
        motor_encoder_indexes_(i) =
            motors_[i]->get_measurement(mi::encoder_index)->length() > 0
                ? motors_[i]
                      ->get_measurement(mi::encoder_index)
                      ->newest_element()
                : std::nan("");
    }
    // acquire the actual motor torques
    motor_torques_ = motor_currents_.array() * motor_torque_constants_.array();
    // acquire the last sent motor torques
    motor_target_torques_ =
        motor_target_currents_.array() * motor_torque_constants_.array();

    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = motor_positions_.array() / joint_gear_ratios_.array();
    // acquire the joint velocities
    joint_velocities_ = motor_velocities_.array() / joint_gear_ratios_.array();
    // acquire the joint torques
    joint_torques_ = motor_torques_.array() * joint_gear_ratios_.array();
    // acquire the target joint torques
    joint_target_torques_ =
        motor_target_torques_.array() * joint_gear_ratios_.array();
    joint_encoder_index_ =
        motor_encoder_indexes_.array() / joint_gear_ratios_.array();

    /**
     * Boom data
     */
    for (unsigned i = 0; i < boom_encoders_.size(); ++i)
    {
        // acquire the boom positions
        boom_positions_(i) =
            boom_encoders_[i]->get_measurement(mi::position)->newest_element() -
            boom_zero_positions_(i);
        // acquire the boom velocities
        boom_velocities_(i) =
            boom_encoders_[i]->get_measurement(mi::velocity)->newest_element();
        boom_encoder_index_(i) =
            boom_encoders_[i]->get_measurement(mi::encoder_index)->length() > 0
                ? boom_encoders_[i]
                      ->get_measurement(mi::encoder_index)
                      ->newest_element()
                : std::nan("");
    }
    /**
     * Floating Base data
     */
    // for whatever reason, this does not work all on one line
    // boom_positions_.array() * boom_gear_ratios_.array()*boom_length_.array();
    base_positions_ = boom_positions_.array() * boom_gear_ratios_.array();
    base_positions_ *= boom_length_;
    base_velocities_ = boom_velocities_.array() * boom_gear_ratios_.array();
    base_velocities_ *= boom_length_;

    /**
     * Additional data
     */
    // // acquire the slider positions
    // for (unsigned i=0 ; i < slider_positions_.size() ; ++i)
    // {
    //   // acquire the slider
    //   slider_positions_(i) =
    //   sliders_[i]->get_measurement()->newest_element();
    // }
}

void Stuggihop::set_hardstop2zero_offsets(
    const Eigen::Ref<Vector2d> hardstop2zero_offsets)
{
    joint_hardstop2zero_offsets_ = hardstop2zero_offsets;
}

void Stuggihop::set_start2hardstop_offsets(
    const Eigen::Ref<Vector2d> start2hardstop_offsets)
{
    joint_start2hardstop_offsets_ = start2hardstop_offsets;
}

void Stuggihop::send_target_motor_current(
    const Eigen::Ref<Vector2d> target_motor_current)
{
    // set up the target current
    for (unsigned i = 0; i < motors_.size(); ++i)
    {
        motors_[i]->set_current_target(target_motor_current(i));
    }

    // actually send the torques to the robot
    for (unsigned i = 0; i < motors_.size(); ++i)
    {
        motors_[i]->send_if_input_changed();
    }
}

void Stuggihop::send_target_joint_torque(
    const Eigen::Ref<Vector2d> target_joint_torque)
{
    target_motor_current_tmp_ = target_joint_torque.array() /
                                joint_gear_ratios_.array() /
                                motor_torque_constants_.array();
    send_target_motor_current(target_motor_current_tmp_);
}

}  // namespace blmc_robots
