/**
 * \file common_header.hpp
 * \brief The hardware wrapper of the quadruped
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the TestBench8Motors class which defines the test
 * bench with 8 motors.
 */

#ifndef COMMON_HEADER_HPP
#define COMMON_HEADER_HPP

// read some parameters
#include "yaml_cpp_catkin/yaml_cpp_fwd.hpp"

// For mathematical operation
#include <Eigen/Eigen>

// manage the exit of the program with ctrl+c
#include <signal.h> // manage the ctrl+c signal
#include <atomic> // thread safe flag for application shutdown management

// some real_time_tools in order to have a real time control
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/thread.hpp"

// The robot drivers for building the robot wrapper around.
#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>


namespace blmc_robots
{
/**
 * @brief Vector2d shortcut for the eigen vector of size 1.
 */
typedef Eigen::Matrix<double, 1, 1> Vector1d;

/**
 * @brief Vector2d shortcut for the eigen vector of size 2.
 */
typedef Eigen::Matrix<double, 2, 1> Vector2d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 8.
 */
typedef Eigen::Matrix<double, 8, 1> Vector8d;

/**
 * @brief CanBus_ptr shortcut for the shared pointer CanBus type
 */
typedef std::shared_ptr<blmc_drivers::CanBus> CanBus_ptr;

/**
 * @brief CanBusMotorBoard_Ptr shortcut for the shared pointer CanBus type
 */
typedef std::shared_ptr<blmc_drivers::CanBusMotorBoard> CanBusMotorBoard_ptr;

/**
 * @brief MotorInterface_ptr shortcut for the shared pointer MotorInterface type
 */
typedef std::shared_ptr<blmc_drivers::MotorInterface> MotorInterface_ptr;

/**
 * @brief Motor_ptr shortcut for the shared pointer Motor type
 */
typedef std::shared_ptr<blmc_drivers::Motor> Motor_ptr;

/**
 * @brief SafeMotor_ptr shortcut for the shared pointer SafeMotor type
 */
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;

/**
 * @brief Slider_ptr shortcut for the linear potentiometer analog sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

/**
 * @brief ContactSensor_ptr shortcut for the contact sensor. It is also an 
 * analog sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> ContactSensor_ptr;

/**
 * @brief HeightSensor_ptr shortcut for the height sensor. It is also an analog
 * sensor
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> HeightSensor_ptr;

/**
 * @brief mi, this typedef is used to get the measurements from the blmc api
 */
typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopControl(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s is the id of the signal
 */
void my_handler(int)
{
    StopControl = true;
}

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
void enable_ctrl_c()
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopControl = false;
}

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 * 
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name, Eigen::Ref<Eigen::VectorXd> v)
{
    v_name += ": [";
    rt_printf("%s", v_name.c_str());
    for (int i = 0; i < v.size(); ++i)
    {
        rt_printf("%f, ", v(i));
    }
    rt_printf("]\n");
}

/**
 * @brief This small structure is used for reading the calibration parameters
 * for the calibrations demos.
 * 
 * @tparam ROBOT_TYPE 
 */
template<class ROBOT_TYPE>
struct ThreadCalibrationData{
  ROBOT_TYPE* robot;
  Eigen::Vector2d joint_index_to_zero;

  ThreadCalibrationData(ROBOT_TYPE* robot_in){
    robot = robot_in;
    std::cout << "Loading paramters from "
              << YAML_PARAMS
              << std::endl;
    YAML::Node param = YAML::LoadFile(YAML_PARAMS);
    YAML::ReadParameter(param["hardware_communication"]["calibration"],
                        "index_to_zero_angle", joint_index_to_zero);
  }
}

} // namespace blmc_robots

#endif // COMMON_HEADER_HPP
