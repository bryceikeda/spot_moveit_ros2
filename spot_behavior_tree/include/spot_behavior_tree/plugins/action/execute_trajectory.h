#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using namespace BT;

class ExecuteTrajectory : public RosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
    ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params)
    : RosActionNode<control_msgs::action::FollowJointTrajectory>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({ 
        InputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_msg"), 
        InputPort<std::string>("execute_follow_joint_trajectory_action_name"),
        InputPort<double>("goal_duration_tolerance"),
        InputPort<double>("goal_position_tolerance"),
        InputPort<double>("goal_time_tolerance"),
       });
    }
    
    bool setGoal(Goal& goal) override;

    void onHalt() override;

    BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult& wr) override;
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};


        // execute_follow_joint_trajectory_action_name="/joint_trajectory_controller/follow_joint_trajectory"
        // trajectory_remainder="{joint_trajectory_remainder}"