#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h> 

namespace SetupMTCMoveAlongFrameAxis
{

class SetupMTCMoveAlongFrameAxis : public BT::SyncActionNode     
{
    public:
      SetupMTCMoveAlongFrameAxis(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
    private:
      std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetupMTCMoveAlongFrameAxis>("SetupMTCMoveAlongFrameAxis");
}
}