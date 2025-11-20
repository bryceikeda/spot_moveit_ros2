#include "spot_behavior_tree/plugins/action/set_moveit_pose_stamped_target.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
  SetMoveItPoseStampedTarget::RegisterNodes(factory);
}

namespace SetMoveItPoseStampedTarget{
SetMoveItPoseStampedTarget::SetMoveItPoseStampedTarget(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetMoveItPoseStampedTarget::tick()
{
    geometry_msgs::msg::PoseStamped stamped_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("stamped_pose", stamped_pose))
        throw BT::RuntimeError("SetMoveItPoseStampedTarget -> Missing required input [stamped_pose]");
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("SetMoveItPoseStampedTarget -> Missing required input [move_group]");

    move_group->setPoseTarget(stamped_pose);
    
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetMoveItPoseStampedTarget::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("stamped_pose"),
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
    };
}
}
