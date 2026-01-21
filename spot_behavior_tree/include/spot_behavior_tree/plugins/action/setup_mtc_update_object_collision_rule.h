#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit/task_constructor/task.h>

namespace SetupMTCUpdateObjectCollisionRule
{

class SetupMTCUpdateObjectCollisionRule : public BT::SyncActionNode     
{
    public:
      SetupMTCUpdateObjectCollisionRule(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetupMTCUpdateObjectCollisionRule>("SetupMTCUpdateObjectCollisionRule");
}
}