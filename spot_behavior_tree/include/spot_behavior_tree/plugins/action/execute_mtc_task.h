#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

using namespace BT;

class ExecuteMTCTask : public RosActionNode<moveit_task_constructor_msgs::action::ExecuteTaskSolution>
{
public:
    ExecuteMTCTask(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params)
    : RosActionNode<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({ InputPort<moveit_task_constructor_msgs::msg::Solution>("solution"), InputPort<double>("goal_duration_tolerance")
       });
    }
    
    bool setGoal(Goal& goal) override;

    void onHalt() override;

    BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult& wr) override;
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};

