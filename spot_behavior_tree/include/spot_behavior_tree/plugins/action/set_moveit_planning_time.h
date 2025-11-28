#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 

namespace SetMoveItPlanningTime
{

class SetMoveItPlanningTime : public BT::SyncActionNode     
{
    public:
      SetMoveItPlanningTime(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetMoveItPlanningTime>("SetMoveItPlanningTime");
}
}