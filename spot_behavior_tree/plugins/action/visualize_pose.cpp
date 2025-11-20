#include "spot_behavior_tree/plugins/action/visualize_pose.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

BT_REGISTER_NODES(factory)
{
  VisualizePose::RegisterNodes(factory);
}

namespace VisualizePose{
    VisualizePose::VisualizePose(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus VisualizePose::tick()
    {
        geometry_msgs::msg::PoseStamped stamped_pose; 
        if (!getInput<geometry_msgs::msg::PoseStamped>("stamped_pose", stamped_pose))
            throw BT::RuntimeError("VisualizePose -> Missing required input [stamped_pose]");
        
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
        if (!getInput<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools", visual_tools_))
            throw BT::RuntimeError("VisualizePose -> Missing required input [visual_tools_]");

        visual_tools_->publishAxisLabeled(stamped_pose.pose, "stamped_pose");
        visual_tools_->trigger();

        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList VisualizePose::providedPorts()
    {
        return {
            BT::InputPort<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("stamped_pose")
        };
    }
}
