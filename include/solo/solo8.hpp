/**
 * @file
 * @brief SoloX specialization for Solo8.
 * @author Julian Viereck and others
 * @copyright Copyright (c) 2020, New York University & Max Planck Gesellschaft.
 */

#pragma once

#include "solox.hpp"

namespace solo
{
template <>
void SoloX<8>::initialize_joint_modules();

template <>
std::vector<odri_control_interface::CalibrationMethod>
SoloX<8>::get_calibration_directions() const;

extern template class SoloX<8>;

typedef SoloX<8> Solo8;
}  // namespace solo
