#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H

using namespace blmc_motors
{

typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

} // namespace blmc_motors

#endif // COMMON_HEADER_H
