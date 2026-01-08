#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <future>

namespace PlanMTCTask
{

class PlanMTCTask : public BT::StatefulActionNode
{
public:
    PlanMTCTask(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    std::shared_ptr<moveit::task_constructor::Task> task_;
    std::future<moveit::core::MoveItErrorCode> future_;
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    rclcpp::Logger logger_;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
}

} // namespace PlanMTCTask
