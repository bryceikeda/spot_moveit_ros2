#include "spot_behavior_tree/plugins/action/setup_visual_tools.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>

BT_REGISTER_NODES(factory)
{
  SetupVisualTools::RegisterNodes(factory);
}

namespace SetupVisualTools{
    SetupVisualTools::SetupVisualTools(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupVisualTools::tick()
    {
        // Get inputs
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
            throw BT::RuntimeError("SetupVisualTools -> Missing required input [move_group]");

        std::string base_frame;
        if (!getInput<std::string>("base_frame", base_frame))
            throw BT::RuntimeError("SetupVisualTools -> Missing required input [base_frame]");
        
        std::string visual_tools_topic;
        if (!getInput<std::string>("visual_tools_topic", visual_tools_topic))
            throw BT::RuntimeError("SetupVisualTools -> Missing required input [visual_tools_topic]");

        rclcpp::Node::SharedPtr node_ptr;
        if (!config().blackboard->rootBlackboard()->get("node", node_ptr))
            throw BT::RuntimeError("SetupVisualTools -> Missing rclcpp::Node::SharedPtr 'node' in blackboard");

        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_ptr, base_frame, "/" + visual_tools_topic);

        visual_tools_->deleteAllMarkers();   // Clear any existing markers in RViz
        visual_tools_->loadRemoteControl();  // Enable "Next" button in RViz

        // Set output port
        setOutput("visual_tools", visual_tools_);

        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList SetupVisualTools::providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
            BT::InputPort<std::string>("base_frame"),
            BT::InputPort<std::string>("visual_tools_topic"),
            BT::OutputPort<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools"),
        };
    }
}
