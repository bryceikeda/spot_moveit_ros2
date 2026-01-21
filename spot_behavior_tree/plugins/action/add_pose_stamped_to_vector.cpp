#include "spot_behavior_tree/plugins/action/add_pose_stamped_to_vector.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>


BT_REGISTER_NODES(factory)
{
  AddPoseStampedToVector::RegisterNodes(factory);
}

namespace AddPoseStampedToVector{
    AddPoseStampedToVector::AddPoseStampedToVector(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus AddPoseStampedToVector::tick()
    {
        geometry_msgs::msg::PoseStamped pose;
        if (!getInput("input", pose))
        throw BT::RuntimeError("Missing element");

        std::vector<geometry_msgs::msg::PoseStamped> vec;
        getInput("vector", vec);   // empty if not set

        vec.push_back(pose);
        setOutput("vector", vec);

        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList AddPoseStampedToVector::providedPorts()
    {
        return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("input"),
        BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("vector")
        };
    }
}
