#include "spot_behavior_tree/plugins/action/initialize_mtc_task.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>


BT_REGISTER_NODES(factory)
{
  InitializeMTCTask::RegisterNodes(factory);
}

namespace InitializeMTCTask{
    InitializeMTCTask::InitializeMTCTask(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus InitializeMTCTask::tick()
    {
    // ---- Inputs ----
    std::string task_id;
    if (!getInput("task_id", task_id))
    {
        throw BT::RuntimeError("InitializeMTCTask: Missing required input [task_id]");
    }

    double timeout;
    if (!getInput("timeout", timeout))
    {
        throw BT::RuntimeError("InitializeMTCTask: Missing required input [timeout]");
    }

    bool trajectory_monitoring;
    if (!getInput("trajectory_monitoring", trajectory_monitoring))
    {
        throw BT::RuntimeError(
            "InitializeMTCTask: Missing required input [trajectory_monitoring]");
    }

    // ---- Blackboard node ----
    rclcpp::Node::SharedPtr node_ptr;
    if (!config().blackboard->rootBlackboard()->get("node", node_ptr))
    {
        throw BT::RuntimeError(
            "InitializeMTCTask: Missing rclcpp::Node::SharedPtr 'node' in blackboard");
    }

    // ---- Create task ----
    auto task = std::make_shared<moveit::task_constructor::Task>();
    task->stages()->setName(task_id);
    task->loadRobotModel(node_ptr);

    task->setProperty("timeout", timeout);
    task->setProperty("trajectory_monitoring", trajectory_monitoring);

    // ---- Output ----
    setOutput("task", task);

    return BT::NodeStatus::SUCCESS;
    }


    BT::PortsList InitializeMTCTask::providedPorts()
    {
        return {
            BT::InputPort<std::string>("task_id"),
            // BT::InputPort<std::vector<std::string>>("controller_names"),
            BT::InputPort<double>("timeout"),
            BT::InputPort<bool>("trajectory_monitoring"),
            BT::OutputPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
        };
    }
}