#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <behaviortree_cpp/bt_factory.h> 

namespace SetupMoveGroup
{

class SetupMoveGroup : public BT::SyncActionNode     
{
    public:
      SetupMoveGroup(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();

    private:
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
      std::string planning_group_name;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetupMoveGroup>("SetupMoveGroup");
}
}