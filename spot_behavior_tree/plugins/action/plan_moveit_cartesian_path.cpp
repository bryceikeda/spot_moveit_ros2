#include "spot_behavior_tree/plugins/action/plan_moveit_cartesian_path.h"
#include "behaviortree_ros2/plugins.hpp"
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
  PlanMoveItCartesianPath::RegisterNodes(factory);
}

namespace PlanMoveItCartesianPath{
PlanMoveItCartesianPath::PlanMoveItCartesianPath(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus PlanMoveItCartesianPath::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("PlanMoveItCartesianPath -> Missing required input [move_group]");

    std::vector<geometry_msgs::msg::PoseStamped> stamped_waypoints;
    if (!getInput<std::vector<geometry_msgs::msg::PoseStamped>>("stamped_waypoints", stamped_waypoints))
        throw BT::RuntimeError("PlanMoveItCartesianPath -> Missing required input [stamped_waypoints]");

    double eef_step; 
    if (!getInput<double>("eef_step", eef_step))
        throw BT::RuntimeError("PlanMoveItCartesianPath -> Missing required input [eef_step]");

    double jump_threshold; 
    if (!getInput<double>("jump_threshold", jump_threshold))
        throw BT::RuntimeError("PlanMoveItCartesianPath -> Missing required input [jump_threshold]");

    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (std::size_t i = 0; i < stamped_waypoints.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("PlanMoveItCartesianPath"), "Adding to vector.");
        waypoints.push_back(stamped_waypoints[i].pose);
    }
    

    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, robot_trajectory_msg);

    setOutput("robot_trajectory_msg", robot_trajectory_msg);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList PlanMoveItCartesianPath::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("stamped_waypoints"),
        BT::InputPort<double>("eef_step", 0.01, "Step size for each interpolation"),
        BT::InputPort<double>("jump_threshold", 0.0, "The joint-space difference between consecutive waypoints"),
        BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("robot_trajectory_msg")
    };
}
}

