#include "spot_behavior_tree/plugins/action/visualize_waypoints.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

BT_REGISTER_NODES(factory)
{
  VisualizeWaypoints::RegisterNodes(factory);
}

namespace VisualizeWaypoints{
    VisualizeWaypoints::VisualizeWaypoints(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus VisualizeWaypoints::tick()
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints; 
        if (!getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", waypoints))
            throw BT::RuntimeError("VisualizeWaypoints -> Missing required input [waypoints]");
        
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
        if (!getInput<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools", visual_tools_))
            throw BT::RuntimeError("VisualizeWaypoints -> Missing required input [visual_tools_]");


        for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools_->publishAxisLabeled(waypoints[i].pose, "pt" + std::to_string(i), rviz_visual_tools::SMALL);

        visual_tools_->trigger();
        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList VisualizeWaypoints::providedPorts()
    {
        return {
            BT::InputPort<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools"),
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints")
        };
    }
}
