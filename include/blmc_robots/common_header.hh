#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H

#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>

namespace blmc_motors
{

typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

} // namespace blmc_motors

#endif // COMMON_HEADER_H
