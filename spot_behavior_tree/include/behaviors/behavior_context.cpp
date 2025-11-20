#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <memory>
#include <string>

namespace my_project::behaviors
{

/**
 * @brief Shared resources for custom behaviors.
 *
 * This struct mirrors moveit_studio::behaviors::BehaviorContext but allows you to
 * add your own shared resources (e.g., custom loggers, planners, data stores, etc.).
 */
struct CustomBehaviorContext
{
  /// Shared node used by all behaviors
  const std::shared_ptr<rclcpp::Node> node;

  /// Callback groups
  const std::shared_ptr<rclcpp::CallbackGroup> callback_group_mutually_exclusive;
  const std::shared_ptr<rclcpp::CallbackGroup> reentrant_callback_group;

  /// TF2 resources
  std::shared_ptr<tf2_ros::Buffer> transform_buffer_ptr;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_ptr;

  /// Optional MoveIt RobotModel
  std::shared_ptr<moveit::core::RobotModel> robot_model;

  /// 🧩 Example: your own shared resource
  std::string shared_data_path;
  std::shared_ptr<rclcpp::Logger> custom_logger;

  /**
   * @brief Constructor
   * @param node_in Shared ROS2 node
   * @param tf2_spin_thread Whether to start TF2 spin thread
   * @param load_robot_model Whether to load the MoveIt robot model
   */
  CustomBehaviorContext(const std::shared_ptr<rclcpp::Node>& node_in,
                        bool tf2_spin_thread = true,
                        bool load_robot_model = false)
    : node(node_in),
      callback_group_mutually_exclusive(
          node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
      reentrant_callback_group(
          node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
      custom_logger(std::make_shared<rclcpp::Logger>(node->get_logger())),
      shared_data_path("/tmp/my_behavior_data/")
  {
    // Initialize TF2
    transform_buffer_ptr = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    transform_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*transform_buffer_ptr, node, tf2_spin_thread);
    transform_broadcaster_ptr = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // Optionally load the robot model
    if (load_robot_model)
    {
      try
      {
        robot_model = moveit::core::RobotModelLoader("robot_description").getModel();
        if (!robot_model)
        {
          RCLCPP_WARN(node->get_logger(), "Failed to load robot model. robot_description may be missing.");
        }
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(node->get_logger(), "Exception while loading robot model: %s", e.what());
      }
    }
  }

  // Deleted copy and move semantics (same as original)
  CustomBehaviorContext(const CustomBehaviorContext&) = delete;
  CustomBehaviorContext& operator=(const CustomBehaviorContext&) = delete;
  CustomBehaviorContext(CustomBehaviorContext&&) = delete;
  CustomBehaviorContext& operator=(CustomBehaviorContext&&) = delete;
};

}  // namespace my_project::behaviors
