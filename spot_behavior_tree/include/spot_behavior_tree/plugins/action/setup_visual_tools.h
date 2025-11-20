#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <behaviortree_cpp/bt_factory.h> 

namespace SetupVisualTools
{

class SetupVisualTools : public BT::SyncActionNode     
{
    public:
      SetupVisualTools(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();

    private:
      moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetupVisualTools>("SetupVisualTools");
}
}