/**
 * \file solo.cpp
 * \brief Execute the main program to control the solo
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include <sstream>

#include "solo/dynamic_graph_manager/dgm_solo12.hpp"

int main()
{
    // Get the dynamic_graph_manager config file.
    std::string robot_properties_yaml_path = ROBOT_PROPERTIES_YAML_PATH;
    std::cout << "Loading paramters from " << robot_properties_yaml_path
              << std::endl;
    YAML::Node param = YAML::LoadFile(robot_properties_yaml_path);

    // Create the dgm.
    solo::DGMSolo12 dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();

    // Wait until ROS is shutdown.
    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}
