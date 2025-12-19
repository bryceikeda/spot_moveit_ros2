#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <control_msgs/action/gripper_command.hpp>

using namespace BT;

class MoveGripperAction : public RosActionNode<control_msgs::action::GripperCommand>
{
public:
    MoveGripperAction(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params)
    : RosActionNode<control_msgs::action::GripperCommand>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({ InputPort<double>("position")
       });
    }
    
    bool setGoal(Goal& goal) override;

    void onHalt() override;

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;


};