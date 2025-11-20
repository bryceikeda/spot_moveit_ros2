#include "spot_behavior_tree/plugins/action/set_moveit_joint_value_target.h"
#include <moveit/move_group_interface/move_group_interface.h>

BT_REGISTER_NODES(factory)
{
  SetMoveitJointValueTarget::RegisterNodes(factory);
}

namespace SetMoveitJointValueTarget{
SetMoveitJointValueTarget::SetMoveitJointValueTarget(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetMoveitJointValueTarget::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("SetMoveitJointValueTarget -> Missing required input [move_group]");

    bool within_bounds; 

    sensor_msgs::msg::JointState joint_state;
    if (getInput("joint_state", joint_state))
    {
        within_bounds = move_group->setJointValueTarget(joint_state);
        if (!within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("SetMoveitJointValueTarget"), "Target joint position(s) were outside of limits; planning will clamp values.");
        }
        return BT::NodeStatus::SUCCESS;
    }

    std::vector<double> joint_values;
    if (getInput("joint_values", joint_values))
    {
        within_bounds = move_group->setJointValueTarget(joint_values);
        if (!within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("SetMoveitJointValueTarget"), "Target joint position(s) were outside of limits; planning will clamp values.");
        }
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList SetMoveitJointValueTarget::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::InputPort<sensor_msgs::msg::JointState>("joint_state"),
        BT::InputPort<std::vector<double>>("joint_values"),
    };
}
}
