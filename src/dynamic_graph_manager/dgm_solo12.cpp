/**
 * \file dgm_solo.cpp
 * \brief The hardware wrapper of the solo robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "solo/dynamic_graph_manager/dgm_solo12.hpp"
#include "dynamic_graph_manager/ros.hpp"

namespace solo
{
DGMSolo12::DGMSolo12()
{
    was_in_safety_mode_ = false;
}

DGMSolo12::~DGMSolo12()
{
}

void DGMSolo12::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters.
     */
    solo::Vector8d joint_index_to_zero;
    YAML::ReadParameter(params_["hardware_communication"]["calibration"],
                        "index_to_zero_angle",
                        zero_to_index_angle_from_file_);

    // Get the hardware communication ros node handle.
    dynamic_graph_manager::RosNodePtr ros_node_handle =
        dynamic_graph_manager::get_ros_node(
            dynamic_graph_manager::HWC_ROS_NODE_NAME);

    /** Initialize the user commands. */
    ros_user_commands_.push_back(
        ros_node_handle->create_service<mim_msgs::srv::JointCalibration>(
            "calibrate_joint_position",
            std::bind(&DGMSolo12::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));

    std::string network_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id", network_id);

    std::string serial_port;
    YAML::ReadParameter(
        params_["hardware_communication"], "serial_port", serial_port);

    solo_.initialize(network_id, serial_port);
}

bool DGMSolo12::is_in_safety_mode()
{
    // Check if any card is in an error state.
    if (solo_.has_error()) {
      was_in_safety_mode_ = true;
      static int counter = 0;
      if (counter % 2000 == 0) {
        printf("DGMSolo12: Going into safe mode as motor card reports error.\n");
      }
      counter += 1;
    }

    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode())
    {
      static int counter = 0;
      was_in_safety_mode_ = true;
      if (counter % 2000 == 0)
      {
        printf("DGMSolo12: is_in_safety_mode.\n");
      }
      counter++;
    }
    return was_in_safety_mode_;
  }

  void DGMSolo12::compute_safety_controls()
  {
    // Check if there is an error with the motors. If so, best we can do is
    // to command zero torques.
    if (solo_.has_error()) {
      for (auto ctrl = motor_controls_map_.begin();
          ctrl != motor_controls_map_.end();
          ++ctrl)
      {
          ctrl->second.fill(0.0);
      }
    } else {
      // The motors are fine.
      // --> Run a D controller to damp the current motion.
      motor_controls_map_.at("ctrl_joint_torques") =
          -0.05 * sensors_map_.at("joint_velocities");
    }
  }

void DGMSolo12::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    solo_.acquire_sensors();

    /**
     * Joint data.
     */
    map.at("joint_positions") = solo_.get_joint_positions();
    map.at("joint_velocities") = solo_.get_joint_velocities();
    map.at("joint_torques") = solo_.get_joint_torques();
    map.at("joint_target_torques") = solo_.get_joint_target_torques();
    map.at("joint_encoder_index") = solo_.get_joint_encoder_index();

    /**
     * Additional data.
     */
    map.at("slider_positions") = solo_.get_slider_positions();
    map.at("imu_accelerometer") = solo_.get_imu_accelerometer();
    map.at("imu_gyroscope") = solo_.get_imu_gyroscope();
    map.at("imu_attitude") = solo_.get_imu_attitude();
    map.at("imu_linear_acceleration") = solo_.get_imu_linear_acceleration();

    /**
     * Robot status.
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    const std::array<bool, 12>& motor_enabled = solo_.get_motor_enabled();
    const std::array<bool, 12>& motor_ready = solo_.get_motor_ready();
    const std::array<bool, 6>& motor_board_enabled =
        solo_.get_motor_board_enabled();
    const std::array<int, 6>& motor_board_errors =
        solo_.get_motor_board_errors();

    for (size_t i = 0; i < motor_enabled.size(); ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (size_t i = 0; i < motor_board_enabled.size(); ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }
}

void DGMSolo12::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        // Here we need to perform an internal copy. Otherwise the compiler
        // complains.
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        // Actually send the control to the robot.
        solo_.send_target_joint_torque(ctrl_joint_torques_);

        ctrl_joint_positions_ = map.at("ctrl_joint_positions");
        solo_.send_target_joint_position(ctrl_joint_positions_);

        ctrl_joint_velocities_ = map.at("ctrl_joint_velocities");
        solo_.send_target_joint_velocity(ctrl_joint_velocities_);

        joint_position_gains_ = map.at("ctrl_joint_position_gains");
        solo_.send_target_joint_position_gains(joint_position_gains_);

        joint_velocity_gains_ = map.at("ctrl_joint_velocity_gains");
        solo_.send_target_joint_velocity_gains(joint_velocity_gains_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMSolo12::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

void DGMSolo12::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // Parse and register the command for further call.
    add_user_command(std::bind(&DGMSolo12::calibrate_joint_position,
                               this,
                               zero_to_index_angle_from_file_));

    // Return a sanity check that assert that the function has been correctly
    // registered in the hardware process.
    res->sanity_check = true;
}

void DGMSolo12::calibrate_joint_position(
    const solo::Vector12d& zero_to_index_angle)
{
    solo_.request_calibration(zero_to_index_angle);
}

}  // namespace solo
