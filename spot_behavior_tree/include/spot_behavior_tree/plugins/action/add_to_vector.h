#pragma once

#include <behaviortree_cpp/action_node.h>
#include <vector>
#include "behaviortree_ros2/plugins.hpp"
#include <rclcpp/rclcpp.hpp>

template <typename T>
class AddToVector : public BT::SyncActionNode
{
public:
    AddToVector(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<T>("input"),
            BT::BidirectionalPort<std::vector<T>>("vector"),
        };
    }

    BT::NodeStatus tick() override
    {
        T input;
        if (!getInput("input", input))
            throw BT::RuntimeError("AddToVector: missing input [input]");

        std::vector<T> vec;
        if (!getInput("vector", vec))
        {
            vec = std::vector<T>();
            RCLCPP_ERROR(rclcpp::get_logger("AddToVector"), "No vector found, creating a new one.");
        }
        
        vec.push_back(input);
        setOutput("vector", vec);

        return BT::NodeStatus::SUCCESS;
    }
};