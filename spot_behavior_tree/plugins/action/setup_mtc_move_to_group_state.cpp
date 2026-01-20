#include "spot_behavior_tree/plugins/action/setup_mtc_move_to_group_state.h"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/task.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/stages/move_to.h>
#include "spot_behavior_tree/string_vector.h"
#include <moveit/task_constructor/stages/generate_grasp_pose.h>

using namespace moveit::task_constructor;

BT_REGISTER_NODES(factory)
{
    SetupMTCMoveToGroupState::RegisterNodes(factory);
}
auto logger = rclcpp::get_logger("SetupMTCMoveToGroupState");

namespace SetupMTCMoveToGroupState
{
SetupMTCMoveToGroupState::SetupMTCMoveToGroupState(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList SetupMTCMoveToGroupState::providedPorts()
{
    return {
        BT::InputPort<std::string>("group_state"),
        BT::InputPort<std::string>("planning_group_name"),
        BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
    };
}

BT::NodeStatus SetupMTCMoveToGroupState::tick()
{
    // ---------------- Inputs ----------------
    std::string group_state, planning_group_name;
    std::shared_ptr<moveit::task_constructor::Task> task;

    getInput("group_state", group_state);
    getInput("planning_group_name", planning_group_name);
    getInput("task", task);
    
    auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

    auto stage = std::make_unique<stages::MoveTo>("move to group state", joint_interpolation);
    stage->setGroup(planning_group_name);
    stage->setGoal(group_state);
    task->add(std::move(stage));

    // ---------------- Add to task ----------------
    // ---------------- Output ----------------
    setOutput("task", task);
    return BT::NodeStatus::SUCCESS;
}
} // namespace SetupMTCMoveToGroupState