#include "spot_behavior_tree/plugins/action/attach_object_to_link.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include "spot_behavior_tree/string_vector.h"

BT_REGISTER_NODES(factory)
{
  AttachObjectToLink::RegisterNodes(factory);
}

namespace AttachObjectToLink{
AttachObjectToLink::AttachObjectToLink(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus AttachObjectToLink::tick()
{
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    if (!getInput<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group))
        throw BT::RuntimeError("AttachObjectToLink -> Missing required input [move_group]");

    std::string link;
    if (!getInput<std::string>("link", link))
        throw BT::RuntimeError("AttachObjectToLink -> Missing required input [link]");
    
    std::string object_id;
    if (!getInput<std::string>("object_id", object_id))
        throw BT::RuntimeError("AttachObjectToLink -> Missing required input [object_id]");
            
    std::vector<std::string> touch_links;
    if (!getInput<std::vector<std::string>>("touch_links", touch_links))
        throw BT::RuntimeError("AttachObjectToLink -> Missing required input [touch_links]");

    move_group->attachObject(object_id, link, touch_links);
    return BT::NodeStatus::SUCCESS;

}

BT::PortsList AttachObjectToLink::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group"),
        BT::InputPort<std::vector<std::string>>("touch_links"),
        BT::InputPort<std::string>("link"),
        BT::InputPort<std::string>("object_id"),
    };
}
}
