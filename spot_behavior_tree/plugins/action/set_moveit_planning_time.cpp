#include "spot_behavior_tree/plugins/action/set_moveit_planning_time.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>

BT_REGISTER_NODES(factory)
{
  SetMoveItPlanningTime::RegisterNodes(factory);
}

namespace SetMoveItPlanningTime{
SetMoveItPlanningTime::SetMoveItPlanningTime(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetMoveItPlanningTime::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("SetMoveItPlanningTime -> Missing required input [move_group]");

    double seconds;
    if (!getInput<double>("seconds", seconds))
        throw BT::RuntimeError("SetMoveItPlanningTime -> Missing required input [seconds]");

    move_group->setPlanningTime(seconds);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetMoveItPlanningTime::providedPorts()
{
    return {
        BT::InputPort<double>("seconds"),
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
    };
}
}
