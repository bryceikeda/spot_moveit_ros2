#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit/task_constructor/task.h>

namespace SetupMTCIgnoreCollisionsBetweenObjects
{

class SetupMTCIgnoreCollisionsBetweenObjects : public BT::SyncActionNode     
{
    public:
      SetupMTCIgnoreCollisionsBetweenObjects(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetupMTCIgnoreCollisionsBetweenObjects>("SetupMTCIgnoreCollisionsBetweenObjects");
}
}