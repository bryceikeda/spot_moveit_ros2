#include "spot_behavior_tree/plugins/action/execute_trajectory.h"
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>

bool ExecuteTrajectory::setGoal(RosActionNode::Goal& goal)
{
    if (!getInput("joint_trajectory_msg", goal.trajectory))
        throw BT::RuntimeError("ExecuteTrajectory -> Missing required input [joint_trajectory]");

    double path_tolerance_val;
    if (getInput("path_tolerance", path_tolerance_val))
    {
        control_msgs::msg::JointTolerance tol;
        tol.name = "";               // empty = applies to all joints
        tol.position = path_tolerance_val;
        tol.velocity = 0.0;
        tol.acceleration = 0.0;
        goal.path_tolerance.push_back(tol);
    }

    // --- Optional goal tolerances ---
    double goal_tolerance_val;
    if (getInput("goal_tolerance", goal_tolerance_val))
    {
        control_msgs::msg::JointTolerance tol;
        tol.name = "";               // empty = applies to all joints
        tol.position = goal_tolerance_val;
        tol.velocity = 0.0;
        tol.acceleration = 0.0;
        goal.goal_tolerance.push_back(tol);
    }

    // --- Optional goal time tolerance ---
    double goal_time_sec;
    if (getInput("goal_time_tolerance", goal_time_sec))
    {
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(goal_time_sec);
    }
    return true;
}


NodeStatus ExecuteTrajectory::onResultReceived(const RosActionNode::WrappedResult& wr)
{
    int ec = wr.result->error_code;

    // Check the error code from the action result
    if (ec == control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL)
    {
        return NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ExecuteTrajectory"),
                        "ExecuteTrajectory failed with MoveIt error code %d and message: %s",
                        wr.result->error_code, wr.result->error_string.c_str());
        return NodeStatus::FAILURE;
    }
}

NodeStatus ExecuteTrajectory::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure: %s", name().c_str(), toStr(error));
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExecuteTrajectory::onFeedback(
    const std::shared_ptr<const Feedback> feedback)
{
    return BT::NodeStatus::RUNNING;
}

void ExecuteTrajectory::onHalt()
{
    RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(ExecuteTrajectory, "ExecuteTrajectory");