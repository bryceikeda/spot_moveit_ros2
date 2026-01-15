#include "spot_behavior_tree/plugins/action/add_to_string_vector.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>


BT_REGISTER_NODES(factory)
{
  AddToStringVector::RegisterNodes(factory);
}

namespace AddToStringVector{
    AddToStringVector::AddToStringVector(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus AddToStringVector::tick()
    {
        std::string input;
        if (!getInput("element", input))
        throw BT::RuntimeError("Missing element");

        std::vector<std::string> vec;
        getInput("vector", vec);   // empty if not set

        vec.push_back(input);
        setOutput("vector", vec);

        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList AddToStringVector::providedPorts()
    {
        return {
        BT::InputPort<std::string>("element"),
        BT::BidirectionalPort<std::vector<std::string>>("vector")
        };
    }
}


