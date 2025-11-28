#include "spot_behavior_tree/plugins/action/visualize_trajectory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

BT_REGISTER_NODES(factory)
{
    VisualizeTrajectory::RegisterNodes(factory);
}

namespace VisualizeTrajectory {

VisualizeTrajectory::VisualizeTrajectory(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{}

// Main tick
BT::NodeStatus VisualizeTrajectory::tick()
{
    // Get the visual tools
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    if (!getInput<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools", visual_tools_))
        throw BT::RuntimeError("VisualizeTrajectory -> Missing required input [visual_tools]");

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("SetupVisualTools -> Missing required input [move_group]");

    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    if (!getInput<moveit_msgs::msg::RobotTrajectory>("robot_trajectory_msg", robot_trajectory))
        throw BT::RuntimeError("VisualizeTrajectory -> Missing required input [robot_trajectory_msg]");

    // Safely get RobotModel and JointModelGroup
    moveit::core::RobotModelConstPtr robot_model = move_group->getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(move_group->getName());

    // Publish trajectory
    visual_tools_->publishTrajectoryLine(robot_trajectory, joint_model_group);
    visual_tools_->trigger();

    return BT::NodeStatus::SUCCESS;
}

// Define input ports
BT::PortsList VisualizeTrajectory::providedPorts()
{
    return {
        BT::InputPort<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools"),
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::InputPort<moveit_msgs::msg::RobotTrajectory>("robot_trajectory_msg")
    };
}
} // namespace VisualizeTrajectory
