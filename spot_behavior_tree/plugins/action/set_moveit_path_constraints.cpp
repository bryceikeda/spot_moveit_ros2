#include "spot_behavior_tree/plugins/action/set_moveit_pose_stamped_target.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>

BT_REGISTER_NODES(factory)
{
  SetMoveItPathConstraints::RegisterNodes(factory);
}

namespace SetMoveItPathConstraints{
SetMoveItPathConstraints::SetMoveItPathConstraints(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetMoveItPathConstraints::tick()
{
    moveit_msgs::msg::Constraints constraints;
    if (!getInput<moveit_msgs::msg::Constraints>("constraints", constraints))
        throw BT::RuntimeError("SetMoveItPathConstraints -> Missing required input [constraints]");
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("SetMoveItPathConstraints -> Missing required input [move_group]");

    move_group->setPathConstraints(constraints);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetMoveItPathConstraints::providedPorts()
{
    return {
        BT::InputPort<moveit_msgs::msg::Constraints>("constraints"),
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
    };
}
}
