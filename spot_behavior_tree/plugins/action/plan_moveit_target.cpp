#include "spot_behavior_tree/plugins/action/plan_moveit_target.h"
#include "behaviortree_ros2/plugins.hpp"

BT_REGISTER_NODES(factory)
{
  PlanMoveItTarget::RegisterNodes(factory);
}

namespace PlanMoveItTarget{
PlanMoveItTarget::PlanMoveItTarget(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus PlanMoveItTarget::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("PlanMoveItTarget -> Missing required input [move_group]");

    bool plan_success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!plan_success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Planning failed for the target pose.");
        return BT::NodeStatus::FAILURE;
    }
    setOutput("robot_trajectory_msg", plan.trajectory_);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList PlanMoveItTarget::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("robot_trajectory_msg")
    };
}
}