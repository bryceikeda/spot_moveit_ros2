#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit/task_constructor/task.h>

namespace InitializeMTCTask
{

class InitializeMTCTask : public BT::SyncActionNode     
{
    public:
      InitializeMTCTask(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
    private:
      std::shared_ptr<moveit::task_constructor::Task> task; 
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
}
}