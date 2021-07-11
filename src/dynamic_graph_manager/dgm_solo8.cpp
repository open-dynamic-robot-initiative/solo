/**
 * \file dgm_solo.cpp
 * \brief The hardware wrapper of the solo robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "solo/dynamic_graph_manager/dgm_solo8.hpp"

#ifdef BUILD_WITH_ROS
#include "dynamic_graph_manager/ros.hpp"
#endif

namespace solo
{
DGMSolo8::DGMSolo8()
{
    was_in_safety_mode_ = false;
}

DGMSolo8::~DGMSolo8()
{
}

void DGMSolo8::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters
     */
    solo::Vector8d joint_index_to_zero;
    YAML::ReadParameter(params_["hardware_communication"]["calibration"],
                        "index_to_zero_angle",
                        zero_to_index_angle_from_file_);

#ifdef BUILD_WITH_ROS
    // get the hardware communication ros node handle
    dynamic_graph_manager::RosNodePtr ros_node_handle =
        dynamic_graph_manager::get_ros_node(
            dynamic_graph_manager::HWC_ROS_NODE_NAME);

    /** initialize the user commands */
    ros_user_commands_.push_back(
        ros_node_handle->create_service<mim_msgs::srv::JointCalibration>(
            "calibrate_joint_position",
            std::bind(&DGMSolo8::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));
#endif

    std::string network_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id", network_id);

    solo_.initialize(network_id);
}

//  bool DGMSolo8::is_in_safety_mode()
//  {
//    was_in_safety_mode_ |= solo_.get_joint_velocities().cwiseAbs().maxCoeff()
//    > 100000003.875; if (was_in_safety_mode_ ||
//    DynamicGraphManager::is_in_safety_mode()) {
//      was_in_safety_mode_ = true;
//      printf("Killing robot because velocity limit exceeded...\n");
//      return true;
//   } else {
//      return false;
//    }
//  }

void DGMSolo8::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    solo_.acquire_sensors();

    /**
     * Joint data
     */
    map.at("joint_positions") = solo_.get_joint_positions();
    map.at("joint_velocities") = solo_.get_joint_velocities();
    map.at("joint_torques") = solo_.get_joint_torques();
    map.at("joint_target_torques") = solo_.get_joint_target_torques();
    map.at("joint_encoder_index") = solo_.get_joint_encoder_index();

    /**
     * Additional data
     */
    map.at("contact_sensors") = solo_.get_contact_sensors_states();
    map.at("slider_positions") = solo_.get_slider_positions();

    /**
     * Robot status
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    const std::array<bool, 8>& motor_enabled = solo_.get_motor_enabled();
    const std::array<bool, 8>& motor_ready = solo_.get_motor_ready();
    const std::array<bool, 4>& motor_board_enabled =
        solo_.get_motor_board_enabled();
    const std::array<int, 4>& motor_board_errors =
        solo_.get_motor_board_errors();

    for (unsigned i = 0; i < 8; ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (unsigned i = 0; i < 4; ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }
}

void DGMSolo8::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        // here we need to perform and internal copy. Otherwise the compilator
        // complains
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        // Actually send the control to the robot
        solo_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMSolo8::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

#ifdef BUILD_WITH_ROS
void DGMSolo8::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMSolo8::calibrate_joint_position,
                               this,
                               zero_to_index_angle_from_file_));

    // return whatever the user want
    res->sanity_check = true;
}
#endif

void DGMSolo8::calibrate_joint_position(
    const solo::Vector8d& zero_to_index_angle)
{
    solo_.request_calibration(zero_to_index_angle);
}

}  // namespace solo
