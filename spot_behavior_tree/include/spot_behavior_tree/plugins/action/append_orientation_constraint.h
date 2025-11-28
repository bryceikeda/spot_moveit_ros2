#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_msgs/msg/constraints.hpp>

namespace AppendOrientationConstraint
{

class AppendOrientationConstraint : public BT::SyncActionNode     
{
    public:
      AppendOrientationConstraint(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
      moveit_msgs::msg::Constraints constraints; 
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<AppendOrientationConstraint>("AppendOrientationConstraint");
}
}