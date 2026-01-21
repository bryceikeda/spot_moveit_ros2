#include "spot_behavior_tree/plugins/action/reset_pose_stamped_vector.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
  ResetPoseStampedVector::RegisterNodes(factory);
}

namespace ResetPoseStampedVector{
ResetPoseStampedVector::ResetPoseStampedVector(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus ResetPoseStampedVector::tick()
  {
      std::vector<geometry_msgs::msg::PoseStamped> vec;
      setOutput("vector", vec);
      return BT::NodeStatus::SUCCESS;
  }

  BT::PortsList ResetPoseStampedVector::providedPorts()
  {
      return {
          BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("vector")
      };
  }
}
