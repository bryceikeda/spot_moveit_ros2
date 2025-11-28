#include "spot_behavior_tree/plugins/action/initialize_motion_constraints.h"

BT_REGISTER_NODES(factory)
{
  InitializeMotionConstraints::RegisterNodes(factory);
}

namespace InitializeMotionConstraints{
InitializeMotionConstraints::InitializeMotionConstraints(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMotionConstraints::tick()
{
    setOutput("constraints", constraints);
    return BT::NodeStatus::SUCCESS;

}

BT::PortsList InitializeMotionConstraints::providedPorts()
{
    return {
        BT::OutputPort<moveit_msgs::msg::Constraints>("constraints"),
    };
}
}