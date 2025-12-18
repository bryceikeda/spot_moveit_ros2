#include "spot_behavior_tree/plugins/action/setup_mtc_current_state.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/current_state.h>

BT_REGISTER_NODES(factory)
{
  SetupMTCCurrentSTate::RegisterNodes(factory);
}

namespace SetupMTCCurrentSTate{
    SetupMTCCurrentSTate::SetupMTCCurrentSTate(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupMTCCurrentSTate::tick()
    {
        // ---- Inputs ----
        std::shared_ptr<moveit::task_constructor::Task> task;
        if (!getInput("task", task))
        {
            throw BT::RuntimeError("SetupMTCCurrentSTate: Missing required input [task]");
        }

        auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
        task->add(std::move(current_state));

        // ---- Output ----
        setOutput("task", task);

        return BT::NodeStatus::SUCCESS;
    }


    BT::PortsList SetupMTCCurrentSTate::providedPorts()
    {
        return {
            BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
        };
    }
}
