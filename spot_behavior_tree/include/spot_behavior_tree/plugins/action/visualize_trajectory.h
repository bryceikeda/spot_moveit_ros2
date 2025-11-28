#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h> 

namespace VisualizeTrajectory
{

class VisualizeTrajectory : public BT::SyncActionNode     
{
    public:
      VisualizeTrajectory(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<VisualizeTrajectory>("VisualizeTrajectory");
}
}