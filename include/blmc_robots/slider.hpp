#pragma once

#include <array>

#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>
#include <blmc_drivers/devices/motor.hpp>


namespace blmc_robots
{

class Slider
{
public:

    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;


    Slider(std::shared_ptr<blmc_drivers::AnalogSensorInterface> analog_sensor,
           const double& min_position = 0,
           const double& max_position = 1.0)
    {
        analog_sensor_ = analog_sensor;

        min_position_ = min_position;
        max_position_ = max_position;
    }

    double get_position() const
    {
        auto measurement_history = analog_sensor_->get_measurement();

        if(measurement_history->length() == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        double measurement = measurement_history->newest_element();
        return min_position_ + measurement * (max_position_ - min_position_);
    }

private:

    std::shared_ptr<blmc_drivers::AnalogSensorInterface> analog_sensor_;

    double min_position_;
    double max_position_;
};




template <int COUNT>
class Sliders
{
public:
    typedef Eigen::Matrix<double, COUNT, 1> Vector;


    Sliders(
            const std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, COUNT>& analog_sensors,
            const Vector& min_positions,
            const Vector& max_positions)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            sliders_[i] = std::make_shared<Slider>(analog_sensors[i],
                                                   min_positions[i],
                                                   max_positions[i]);
        }
    }

    Vector get_positions() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = sliders_[i]->get_position();
        }
        return positions;
    }

private:
    std::array<std::shared_ptr<Slider>, COUNT> sliders_;

};




} // namespace blmc_robots
