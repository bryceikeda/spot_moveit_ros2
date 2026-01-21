#include "spot_behavior_tree/plugins/action/execute_mtc_task.h"
#include <rclcpp/rclcpp.hpp>

bool ExecuteMTCTask::setGoal(RosActionNode::Goal& goal)
{
    if (!getInput("solution", goal.solution))
        throw BT::RuntimeError("ExecuteMTCTask -> Missing required input [solution]");
    // --- Optional goal duration tolerance ---
    double goal_duration_tolerance;
    if (getInput("goal_duration_tolerance", goal_duration_tolerance))
    {
        auto node = node_.lock();
        if (!node)
        {
            throw BT::RuntimeError(
                "ExecuteMTCTask -> ROS node expired"
            );
        }

        // Declare parameter if necessary
        if (!node->has_parameter("goal_duration_tolerance"))
        {
            node->declare_parameter(
                "goal_duration_tolerance",
                goal_duration_tolerance
            );
        }

        node->set_parameter(
            rclcpp::Parameter(
                "goal_duration_tolerance",
                goal_duration_tolerance
            )
        );

        RCLCPP_INFO(
            node->get_logger(),
            "Set goal_duration_tolerance = %.3f",
            goal_duration_tolerance
        );
    }

    return true;
}


NodeStatus ExecuteMTCTask::onResultReceived(const RosActionNode::WrappedResult& wr)
{
    int ec = wr.result->error_code.val;

    // Check the error code from the action result
    if (ec == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        return NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ExecuteMTCTask"),
                        "ExecuteMTCTask failed with MoveIt error code %d",
                        wr.result->error_code.val);
        return NodeStatus::FAILURE;
    }
}

NodeStatus ExecuteMTCTask::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure: %s", name().c_str(), toStr(error));
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExecuteMTCTask::onFeedback(
    const std::shared_ptr<const Feedback> feedback)
{
    if (!feedback)
        return BT::NodeStatus::RUNNING;

    std::stringstream ss;
    ss << "[" << name() << "] ExecuteTaskSolution feedback: "
       << "sub_id=" << feedback->sub_id
       << ", sub_no=" << feedback->sub_no;

    RCLCPP_INFO(logger(), "%s", ss.str().c_str());

    return BT::NodeStatus::RUNNING;
}

void ExecuteMTCTask::onHalt()
{
    RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(ExecuteMTCTask, "ExecuteMTCTask");