#include "spot_behavior_tree/plugins/action/reset_vector.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
  ResetVector::RegisterNodes(factory);
}

namespace ResetVector{
ResetVector::ResetVector(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus ResetVector::tick()
{
    std::vector<BT::Any> vec;
    setOutput("vector", vec);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ResetVector::providedPorts()
{
    return {
        BT::OutputPort<std::vector<BT::Any>>("vector")
    };
}
}
