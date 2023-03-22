#include <solo/solo8.hpp>

namespace solo
{
template class SoloX<8>;

template <>
void SoloX<8>::initialize_joint_modules()
{
    odri_control_interface::VectorXi motor_numbers(8);
    motor_numbers << 0, 1, 3, 2, 5, 4, 6, 7;
    odri_control_interface::VectorXb motor_reversed(8);
    motor_reversed << true, true, false, false, true, true, false, false;

    double lHFE = 1.45;
    double lKFE = 2.80;
    Eigen::VectorXd joint_lower_limits(8);
    joint_lower_limits << -lHFE, -lKFE, -lHFE, -lKFE, -lHFE, -lKFE, -lHFE,
        -lKFE;
    Eigen::VectorXd joint_upper_limits(8);
    joint_upper_limits << lHFE, lKFE, lHFE, lKFE, lHFE, lKFE, lHFE, lKFE;

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
SoloX<8>::get_calibration_directions() const
{
    return {odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE,
            odri_control_interface::POSITIVE};
}

}  // namespace solo
