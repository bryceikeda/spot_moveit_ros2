#include "spot_behavior_tree/plugins/action/setup_move_group.h"

BT_REGISTER_NODES(factory)
{
  SetupMoveGroup::RegisterNodes(factory);
}

namespace SetupMoveGroup{
    SetupMoveGroup::SetupMoveGroup(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupMoveGroup::tick()
    {
        if (!getInput<std::string>("planning_group_name", planning_group_name))
            throw BT::RuntimeError("SetupMoveGroup -> Missing required input [planning_group_name]");

        if (!config().blackboard->rootBlackboard()->get("node", node_ptr))
            throw BT::RuntimeError("SetupMoveGroup -> Missing rclcpp::Node::SharedPtr 'node' in blackboard");

        // Construct MoveGroupInterface correctly
        auto move_group =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                node_ptr, planning_group_name);

        setOutput("move_group", move_group);
        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList SetupMoveGroup::providedPorts()
    {
        return {
            BT::InputPort<std::string>("planning_group_name"),
            BT::OutputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group")
        };
    }
}
