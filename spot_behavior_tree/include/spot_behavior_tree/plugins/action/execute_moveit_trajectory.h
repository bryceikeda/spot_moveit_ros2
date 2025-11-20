#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <moveit_msgs/action/execute_trajectory.hpp>

using namespace BT;

class ExecuteMoveItTrajectory : public RosActionNode<moveit_msgs::action::ExecuteTrajectory>
{
public:
    ExecuteMoveItTrajectory(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params)
    : RosActionNode<moveit_msgs::action::ExecuteTrajectory>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({ InputPort<moveit_msgs::msg::RobotTrajectory>("robot_trajectory_msg")
       });
    }
    
    bool setGoal(Goal& goal) override;

    void onHalt() override;

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;

private:
        moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
};