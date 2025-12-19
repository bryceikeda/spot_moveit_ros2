#include "spot_behavior_tree/plugins/action/plan_mtc_task.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

using namespace moveit::task_constructor;

BT_REGISTER_NODES(factory)
{
    PlanMTCTask::RegisterNodes(factory);
}

namespace PlanMTCTask
{
PlanMTCTask::PlanMTCTask(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), logger_(rclcpp::get_logger("PlanMTCTask"))
{}
BT::NodeStatus PlanMTCTask::onStart()
{
    if (!getInput("task", task_))
    {
        throw BT::RuntimeError("PlanMTCTask: Missing required input [task]");
    }

    // Launch planning asynchronously
    future_ = std::async(std::launch::async, [this]() {
        try
        {
            task_->init();
            return task_->plan(1);
        }
        catch (const moveit::task_constructor::InitStageException& e)
        {
            RCLCPP_ERROR(logger_, "Initialization failed: %s", e.what());
            return moveit::core::MoveItErrorCode(moveit::core::MoveItErrorCode::FAILURE);        
        }
    });
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanMTCTask::onRunning()
{
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
        RCLCPP_INFO(logger_, "TICKINGGGG");

        return BT::NodeStatus::RUNNING;
    }

    auto result = future_.get();
    if (result != moveit::core::MoveItErrorCode::SUCCESS ||
        task_->solutions().empty())
    {
        RCLCPP_ERROR(logger_, "Planning failed");
        return BT::NodeStatus::FAILURE;
    }

	moveit_task_constructor_msgs::msg::Solution solution_msg;
	task_->solutions().front()->toMsg(solution_msg);

    task_->introspection().publishSolution(*task_->solutions().front());
    setOutput("solution", solution_msg);
    return BT::NodeStatus::SUCCESS;
}

void PlanMTCTask::onHalted()
{
    RCLCPP_WARN(logger_, "PlanMTCTask halted (planning cannot be cancelled)");
    task_.reset();
}

BT::PortsList PlanMTCTask::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<Task>>("task"),
        BT::OutputPort<moveit_task_constructor_msgs::msg::Solution>("solution"),
    };
}

} // namespace PlanMTCTask