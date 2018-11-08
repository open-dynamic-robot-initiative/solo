/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the Finger
 * \author Manuel Wuthrich
 * \date 2018
 *
 * This file declares the Finger class which defines the test
 * bench with 8 motors.
 */

#pragma once


#include <Eigen/Eigen>
#include <blmc_robots/common_header.hh>

namespace blmc_robots
{

/**
 * @brief The Finger class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class Finger
{
public:

    /**
   * @brief mi this typdef is used to get the measurments form the blmc api
   */
    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

    /**
   * @brief Finger is the constructor of the class.
   */
    Finger();

    /**
   * @brief initialize the robot by setting alining the motors and calibrate the
   * sensors to 0
   */
    void initialize();


    enum MotorMeasurementIndexing {current, position, velocity, encoder_index,
                                   motor_measurement_count};
    enum MotorIndexing {base, center, tip, motor_count};

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    double get_motor_measurement(const int& motor_index,
                           const int& measurement_index) const
    {
        auto measurement_history =
                motors_[motor_index]->get_measurement(measurement_index);

        if(measurement_history->length() == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return measurement_history->newest_element();
    }


    /**
   * @brief send_target_torques sends the target currents to the motors
   */
    void send_target_current(
            const Eigen::Vector3d target_currents);

    /**
   * @brief acquire_sensors acquire all available sensors, WARNING !!!!
   * this method has to be called prior to any gettter to have up to date data.
   */
    void acquire_sensors();



    void send_torques(const Eigen::Vector3d& desired_torques)
    {
        for(size_t i = 0; i < motor_count; i++)
        {
            motors_[i]->set_current_target(desired_torques(i) / gear_ratio_);
        }
        for(size_t i = 0; i < motor_count; i++)
        {
            motors_[i]->send_if_input_changed();
        }
    }


    Eigen::Vector3d get_torques() const
    {
        Eigen::Vector3d torques;

        for(size_t i = 0; i < motor_count; i++)
        {
            torques(i) = get_motor_measurement(i, current) * gear_ratio_;
        }
        return torques;
    }

    Eigen::Vector3d get_positions() const
    {
        Eigen::Vector3d positions;

        for(size_t i = 0; i < motor_count; i++)
        {
            positions(i) = get_motor_measurement(i, position) / gear_ratio_
                    - zero_position_;
        }
        return positions;
    }


    Eigen::Vector3d get_velocities() const
    {
        Eigen::Vector3d velocities;

        for(size_t i = 0; i < motor_count; i++)
        {
            velocities(i) = get_motor_measurement(i, velocity) / gear_ratio_;
        }
        return velocities;
    }


    Eigen::Vector3d get_encoder_indices() const
    {
        Eigen::Vector3d encoder_indices;

        for(size_t i = 0; i < motor_count; i++)
        {
            encoder_indices(i) =
                    get_motor_measurement(i, encoder_index) / gear_ratio_;
        }
        return encoder_indices;
    }




    /**
   * @brief get_motor_velocities
   * @return the current motors velocities (rad/s).
   */
    const Eigen::Vector3d get_motor_velocities()
    {
        return motor_velocities_;
    }

    /**
   * @brief get_motor_currents
   * @return the current motors currents in (Amper: A).
   */
    const Eigen::Vector3d get_motor_currents()
    {
        return motor_currents_;
    }

    /**
   * @brief get_motor_encoder_indexes
   * @return TODO: ask Manuel what is this exactly.
   */
    const Eigen::Vector3d get_motor_encoder_indexes()
    {
        return motor_encoder_indexes_;
    }

    /**
   * @brief get_slider_positions
   * @return the current sliders positions.
   */
    const Eigen::Vector3d get_slider_positions()
    {
        return slider_positions_;
    }

    /**
   * @brief get_max_current
   * @return the maximum current to apply to the motors
   */
    double get_max_current()
    {
        return max_current_;
    }

    /**
   * @brief get_max_current
   * @return the maximum current to apply to the motors
   */
    double get_max_range()
    {
        return max_range_;
    }

private:

    double zero_position_;
    double gear_ratio_;
    double motor_constant_;


    Eigen::Vector3d motor_positions_;
    Eigen::Vector3d motor_velocities_;
    Eigen::Vector3d motor_currents_;
    Eigen::Vector3d motor_encoder_indexes_;
    Eigen::Vector3d slider_positions_;
    Eigen::Vector3d target_currents;

    double max_current_ ;
    double max_range_ ;

    // we have 4 board with each possessing 2 motors and 2 sliders
    std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> motors_;
    std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 3> sliders_;
};

} // namespace blmc_robots
