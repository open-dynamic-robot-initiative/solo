/**
 * @file
 * @brief SoloX specialization for Solo12.
 * @author Julian Viereck and others
 * @copyright Copyright (c) 2019 New York University & Max Planck Gesellschaft
 */
#pragma once

#include "solox.hpp"

namespace solo
{
template <>
void SoloX<12>::initialize_joint_modules();

template <>
std::vector<odri_control_interface::CalibrationMethod>
SoloX<12>::get_calibration_directions() const;

extern template class SoloX<12>;

typedef SoloX<12> Solo12;
}  // namespace solo
