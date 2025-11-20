#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <behaviortree_cpp/bt_factory.h> 

namespace PlanMoveItTarget
{

class PlanMoveItTarget : public BT::SyncActionNode     
{
    public:
        // Original constructor with shared resources
        PlanMoveItTarget(const std::string& name,
                    const BT::NodeConfig& config);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        moveit::planning_interface::MoveGroupInterface::Plan plan;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<PlanMoveItTarget>("PlanMoveItTarget");
}
}