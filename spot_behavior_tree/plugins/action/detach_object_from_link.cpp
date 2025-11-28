#include "spot_behavior_tree/plugins/action/detach_object_from_link.h"
#include <moveit/move_group_interface/move_group_interface.h>

BT_REGISTER_NODES(factory)
{
  DetachObjectFromLink::RegisterNodes(factory);
}

namespace DetachObjectFromLink{
DetachObjectFromLink::DetachObjectFromLink(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus DetachObjectFromLink::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("DetachObjectFromLink -> Missing required input [move_group]");

    std::string object_id;
    if (!getInput<std::string>("object_id", object_id))
        throw BT::RuntimeError("DetachObjectFromLink -> Missing required input [object_id]");

    move_group->detachObject(object_id);
    return BT::NodeStatus::SUCCESS;

}

BT::PortsList DetachObjectFromLink::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::InputPort<std::string>("object_id"),
    };
}
}