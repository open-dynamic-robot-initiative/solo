#include <solo/solo12.hpp>

namespace solo
{
template <>
void SoloX<12>::initialize_joint_modules()
{
    odri_control_interface::VectorXi motor_numbers(12);
    motor_numbers << 0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10;
    odri_control_interface::VectorXb motor_reversed(12);
    motor_reversed << false, true, true, true, false, false, false, true, true,
        true, false, false;

    double lHAA = 0.9;
    double lHFE = 1.45;
    double lKFE = 2.80;
    Eigen::VectorXd joint_lower_limits(12);
    joint_lower_limits << -lHAA, -lHFE, -lKFE, -lHAA, -lHFE, -lKFE, -lHAA,
        -lHFE, -lKFE, -lHAA, -lHFE, -lKFE;
    Eigen::VectorXd joint_upper_limits(12);
    joint_upper_limits << lHAA, lHFE, lKFE, lHAA, lHFE, lKFE, lHAA, lHFE, lKFE,
        lHAA, lHFE, lKFE;

    // Define the joint module.
    joints_ = std::make_shared<odri_control_interface::JointModules>(
        main_board_ptr_,
        motor_numbers,
        motor_torque_constants_(0),
        joint_gear_ratios_(0),
        motor_max_current_(0),
        motor_reversed,
        joint_lower_limits,
        joint_upper_limits,
        80.,
        0.2);
}

template <>
std::vector<odri_control_interface::CalibrationMethod>
SoloX<12>::get_calibration_directions() const
{
    return {odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::NEGATIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::NEGATIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE};
}

template class SoloX<12>;

}  // namespace solo
