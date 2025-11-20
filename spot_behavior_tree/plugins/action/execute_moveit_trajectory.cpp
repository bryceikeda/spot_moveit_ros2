#include "spot_behavior_tree/plugins/action/execute_moveit_trajectory.h"
#include <rclcpp/rclcpp.hpp>

bool ExecuteMoveItTrajectory::setGoal(RosActionNode::Goal& goal)
{
    if (!getInput("robot_trajectory_msg", robot_trajectory_msg))
        throw BT::RuntimeError("ExecuteMoveItTrajectory -> Missing required input [plan]");

    goal.trajectory = robot_trajectory_msg;

    return true;
}

NodeStatus ExecuteMoveItTrajectory::onResultReceived(const RosActionNode::WrappedResult& wr)
{
    int ec = wr.result->error_code.val;

    bool ok = (ec == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

    RCLCPP_INFO(logger(), "%s: ExecuteTrajectory result: %d", name().c_str(), ec);

    return ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus ExecuteMoveItTrajectory::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure: %s", name().c_str(), toStr(error));
    return BT::NodeStatus::FAILURE;
}

void ExecuteMoveItTrajectory::onHalt()
{
    RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(ExecuteMoveItTrajectory, "ExecuteMoveItTrajectory");