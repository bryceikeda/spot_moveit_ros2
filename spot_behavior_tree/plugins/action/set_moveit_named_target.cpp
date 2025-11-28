#include "spot_behavior_tree/plugins/action/set_moveit_named_target.h"
#include <moveit/move_group_interface/move_group_interface.h>

BT_REGISTER_NODES(factory)
{
  SetMoveItNamedTarget::RegisterNodes(factory);
}

namespace SetMoveItNamedTarget{
SetMoveItNamedTarget::SetMoveItNamedTarget(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetMoveItNamedTarget::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("SetMoveItNamedTarget -> Missing required input [move_group]");
    
    std::string target_name;
    if (!getInput<std::string>("target_name", target_name))
        throw BT::RuntimeError("SetMoveItNamedTarget -> Missing required input [target_name]");

    move_group->setNamedTarget(target_name);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetMoveItNamedTarget::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::InputPort<std::string>("target_name"),
    };
}
}
