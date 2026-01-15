#pragma once

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_ros2/plugins.hpp"

namespace TransformPose
{

class TransformPose : public BT::SyncActionNode     
{
    public:
      TransformPose(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<TransformPose>("TransformPose");
}
}