#include "spot_behavior_tree/plugins/action/add_to_vector.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>


BT_REGISTER_NODES(factory)
{
  AddToVector::RegisterNodes(factory);
}

namespace AddToVector{
    AddToVector::AddToVector(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus AddToVector::tick()
    {
        BT::Any input;
        if (!getInput("element", input))
        throw BT::RuntimeError("Missing element");

        std::vector<BT::Any> vec;
        getInput("vector", vec);   // empty if not set

        vec.push_back(input);
        setOutput("vector", vec);

        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList AddToVector::providedPorts()
    {
        return {
        BT::InputPort<BT::Any>("element"),
        BT::BidirectionalPort<std::vector<BT::Any>>("vector")
        };
    }
}


