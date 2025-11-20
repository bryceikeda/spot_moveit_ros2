#include "spot_behavior_tree/plugins/action/create_joint_state.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

BT_REGISTER_NODES(factory)
{
  CreateJointState::RegisterNodes(factory);
}

namespace CreateJointState {

CreateJointState::CreateJointState(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus CreateJointState::tick()
{
    // Get input joint names
    std::vector<std::string> joint_names;
    if (!getInput<std::vector<std::string>>("joint_names", joint_names))
        throw BT::RuntimeError("CreateJointState -> Missing required input [joint_names]");

    // Get input joint positions
    std::vector<double> positions; 
    if (!getInput<std::vector<double>>("positions", positions))
        throw BT::RuntimeError("CreateJointState -> Missing required input [positions]");

    // --- Set the target joint state ---
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = joint_names;
    joint_state.position = positions;

    setOutput("joint_state", joint_state);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList CreateJointState::providedPorts()
{
    return {
        BT::InputPort<std::vector<std::string>>("joint_names"),
        BT::InputPort<std::vector<double>>("positions"),  
        BT::OutputPort<sensor_msgs::msg::JointState>("joint_state")
    };
}

}  // namespace CreateJointState
