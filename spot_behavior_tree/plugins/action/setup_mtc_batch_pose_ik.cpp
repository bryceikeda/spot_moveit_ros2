#include "spot_behavior_tree/plugins/action/setup_mtc_batch_pose_ik.h"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/stages/move_to.h>
#include "spot_behavior_tree/string_vector.h"
#include <moveit/task_constructor/stages/generate_grasp_pose.h>

using namespace moveit::task_constructor;

BT_REGISTER_NODES(factory)
{
    SetupMTCBatchPoseIK::RegisterNodes(factory);
}
auto logger = rclcpp::get_logger("SetupMTCBatchPoseIK");
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
	return Eigen::Translation3d(values[0], values[1], values[2]) *
	       Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

namespace SetupMTCBatchPoseIK
{
SetupMTCBatchPoseIK::SetupMTCBatchPoseIK(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList SetupMTCBatchPoseIK::providedPorts()
{
    return {
        BT::InputPort<int>("max_ik_solutions"),
        BT::InputPort<double>("ik_timeout_s"),
        BT::InputPort<std::string>("end_effector_link"),
        BT::InputPort<std::string>("end_effector_group"),
        BT::InputPort<std::string>("ik_group"),
        BT::InputPort<std::string>("monitored_stage"),
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("target_poses"),
        BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
    };
}

BT::NodeStatus SetupMTCBatchPoseIK::tick()
{
    // ---------------- Inputs ----------------
    int max_ik_solutions = 8;
    double ik_timeout_s = 0.01;
    std::string end_effector_link, end_effector_group, ik_group, monitored_stage;
    std::shared_ptr<moveit::task_constructor::Task> task;
    std::vector<geometry_msgs::msg::PoseStamped> target_poses;

    getInput("max_ik_solutions", max_ik_solutions);
    getInput("ik_timeout_s", ik_timeout_s);
    getInput("end_effector_link", end_effector_link);
    getInput("end_effector_group", end_effector_group);
    getInput("ik_group", ik_group);
    getInput("monitored_stage", monitored_stage);
    getInput("target_poses", target_poses);
    getInput("task", task);
    
    // ---------------- Generate Pose ----------------
    moveit::task_constructor::Stage* monitored_stage_ptr = nullptr;
    monitored_stage_ptr = task->findChild("current state");
    if (!monitored_stage_ptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("SetupMTCBatchPoseIK"),
                     "Monitored stage not found in task current state");
    }

    // push all BT target_poses into the generator
    auto alternatives = std::make_unique<Alternatives>("pose alternatives");

    for (const auto& pose : target_poses)
    {
        auto gen = std::make_unique<stages::GeneratePose>("target pose");
        gen->setPose(pose);
        gen->properties().configureInitFrom(Stage::PARENT);
        gen->setMonitoredStage(monitored_stage_ptr);
        alternatives->add(std::move(gen));
    }

    auto ik = std::make_unique<stages::ComputeIK>("batch IK", std::move(alternatives));
    ik->setMaxIKSolutions(max_ik_solutions);
    ik->setTimeout(ik_timeout_s);
    ik->setMinSolutionDistance(1.0);

    // frame on the end-effector where pose is expressed
    ik->setIKFrame(vectorToEigen({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), end_effector_group); //arm_link_wr1 deleted .2 to add transform pose

    // MTC property plumbing
    ik->properties().set("group", ik_group);              // arm
    ik->properties().set("eef", end_effector_link);     // gripper
    ik->properties().set("marker_ns", "ik_solutions");
    // pull "target_pose" from GeneratePose
    ik->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    
    // ---------------- Add to task ----------------
    task->add(std::move(ik));
    // ---------------- Output ----------------
    setOutput("task", task);
    return BT::NodeStatus::SUCCESS;
}
} // namespace SetupMTCBatchPoseIK