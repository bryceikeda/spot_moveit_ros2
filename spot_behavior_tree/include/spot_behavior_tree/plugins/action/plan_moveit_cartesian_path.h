#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit/move_group_interface/move_group_interface.h>

namespace PlanMoveItCartesianPath
{

class PlanMoveItCartesianPath : public BT::SyncActionNode     
{
    public:
        // Original constructor with shared resources
        PlanMoveItCartesianPath(const std::string& name,
                    const BT::NodeConfig& config);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<PlanMoveItCartesianPath>("PlanMoveItCartesianPath");
}
}