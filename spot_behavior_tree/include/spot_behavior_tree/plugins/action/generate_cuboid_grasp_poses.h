#pragma once

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_ros2/plugins.hpp"

namespace GenerateCuboidGraspPoses
{

class GenerateCuboidGraspPoses : public BT::SyncActionNode     
{
    public:
      GenerateCuboidGraspPoses(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<GenerateCuboidGraspPoses>("GenerateCuboidGraspPoses");
}
}