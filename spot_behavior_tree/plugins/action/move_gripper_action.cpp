#include "spot_behavior_tree/plugins/action/move_gripper_action.h"
#include <rclcpp/rclcpp.hpp>

using control_msgs::action::GripperCommand;

bool MoveGripperAction::setGoal(RosActionNode::Goal& goal)
{
    double position;
    if (!getInput("position", position))
    {
        throw BT::RuntimeError(
            "MoveGripperAction: Missing required input [position]");
    }

    goal.command.position = position;

    RCLCPP_INFO(
        logger(),
        "%s: Sending gripper command (position=%.3f)",
        name().c_str(),
        position);

    return true;
}

NodeStatus MoveGripperAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  const auto& result = wr.result;

  bool success = result->reached_goal;

  RCLCPP_INFO(
      logger(),
      "%s: Gripper result: reached=%d, position=%.3f",
      name().c_str(),
      result->reached_goal,
      result->position);

  return success ? BT::NodeStatus::SUCCESS
                 : BT::NodeStatus::FAILURE;
}

NodeStatus MoveGripperAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(
        logger(),
        "%s: Gripper action failure: %s",
        name().c_str(),
        toStr(error));

    return BT::NodeStatus::FAILURE;
}

void MoveGripperAction::onHalt()
{
    RCLCPP_WARN(logger(), "%s: Gripper command halted", name().c_str());
}

CreateRosNodePlugin(MoveGripperAction, "MoveGripperAction");