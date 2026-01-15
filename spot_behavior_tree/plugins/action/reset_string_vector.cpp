#include "spot_behavior_tree/plugins/action/reset_string_vector.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
  ResetStringVector::RegisterNodes(factory);
}

namespace ResetStringVector{
ResetStringVector::ResetStringVector(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus ResetStringVector::tick()
{
    std::vector<std::string> vec;
    setOutput("vector", vec);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ResetStringVector::providedPorts()
{
    return {
        BT::OutputPort<std::vector<std::string>>("vector")
    };
}
}
