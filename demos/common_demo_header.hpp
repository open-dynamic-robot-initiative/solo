/**
 * @file common_demo_header.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Contains some default tools for creating demos
 * @version 0.1
 * @date 2019-11-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "blmc_robots/common_programs_header.hpp"

namespace blmc_robots
{
/**
 * @brief This small structure is used for reading the calibration parameters
 * for the calibrations demos.
 *
 * @tparam ROBOT_TYPE
 */
template <class ROBOT_TYPE>
struct ThreadCalibrationData
{
    std::shared_ptr<ROBOT_TYPE> robot;
    Eigen::VectorXd joint_index_to_zero;

    ThreadCalibrationData(std::shared_ptr<ROBOT_TYPE> robot_in)
        : robot(robot_in)
    {
        // Reconstruct the path to the DGM yaml file
        std::string shared_directory =
            ament_index_cpp::get_package_share_directory(
                ROBOT_PROPERTIES_PACKAGE_NAME);
        std::string yaml_path =
            shared_directory + "/" + ROBOT_PROPERTIES_YAML_PATH;

        std::cout << "Loading paramters from " << yaml_path << std::endl;
        YAML::Node param = YAML::LoadFile(yaml_path);
        YAML::ReadParameter(param["hardware_communication"]["calibration"],
                            "index_to_zero_angle",
                            joint_index_to_zero);
        std::cout << "joint_index_to_zero: " << joint_index_to_zero
                  << std::endl;
    }
};

}  // namespace blmc_robots