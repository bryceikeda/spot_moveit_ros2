#include "spot_behavior_tree/plugins/action/setup_mtc_plan_to_pose.h"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/stages/move_to.h>
#include "spot_behavior_tree/string_vector.h"

using namespace moveit::task_constructor;

BT_REGISTER_NODES(factory)
{
    SetupMTCPlanToPose::RegisterNodes(factory);
}

namespace SetupMTCPlanToPose
{
SetupMTCPlanToPose::SetupMTCPlanToPose(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList SetupMTCPlanToPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("planning_group_name"),
        BT::InputPort<std::string>("ik_frame"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
        BT::InputPort<int>("max_iterations"),
        BT::InputPort<bool>("keep_orientation"),
        BT::InputPort<std::vector<std::string>>("keep_orientation_link_names"),
        BT::InputPort<double>("keep_orientation_tolerance"),
        BT::InputPort<double>("acceleration_scale_factor"),
        BT::InputPort<double>("velocity_scale_factor"),
        BT::InputPort<std::string>("monitored_stage"),
        BT::InputPort<double>("link_padding"),
        BT::InputPort<unsigned int>("trajectory_sampling_rate"),
        BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
    };
}

BT::NodeStatus SetupMTCPlanToPose::tick()
{
    // ---------------- Inputs ----------------
    std::string group, ik_frame, monitored_stage;
    geometry_msgs::msg::PoseStamped target_pose;
    int max_iterations = 5000;
    bool keep_orientation = false;
    double orientation_tol = 0.1;
    double acceleration_scale_factor = 1.0;
    double velocity_scale_factor = 1.0;
    std::vector<std::string> orientation_links;
    double link_padding = 0.0;
    unsigned int trajectory_sampling_rate = 100;
    std::shared_ptr<moveit::task_constructor::Task> task;

    getInput("planning_group_name", group);
    getInput("ik_frame", ik_frame);
    getInput("target_pose", target_pose);
    getInput("max_iterations", max_iterations);
    getInput("keep_orientation", keep_orientation);
    getInput("keep_orientation_tolerance", orientation_tol);
    getInput("acceleration_scale_factor", acceleration_scale_factor);
    getInput("velocity_scale_factor", velocity_scale_factor);
    getInput("keep_orientation_link_names", orientation_links);
    getInput("monitored_stage", monitored_stage);
    getInput("link_padding", link_padding);
    getInput("trajectory_sampling_rate", trajectory_sampling_rate);
    getInput("task", task);
    rclcpp::Node::SharedPtr node_ptr;
    if (!config().blackboard->rootBlackboard()->get("node", node_ptr))
        throw BT::RuntimeError("SetupMTCPlanToPose -> Missing rclcpp::Node::SharedPtr 'node' in blackboard");

    // ---------------- Generate Pose ----------------
    auto planner = std::make_shared<solvers::PipelinePlanner>(node_ptr, "ompl");
    planner->setMaxVelocityScalingFactor(velocity_scale_factor);
    planner->setMaxAccelerationScalingFactor(acceleration_scale_factor);

    auto connect = std::make_unique<stages::Connect>(
        "connect",
        stages::Connect::GroupPlannerVector{{group, planner}}
    );
    task->add(std::move(connect));

    auto generate_pose = std::make_unique<stages::GeneratePose>("generate target pose");

    moveit::task_constructor::Stage* monitored_stage_ptr = nullptr;
    if (task && task->stages())
    {
        monitored_stage_ptr = task->stages()->findChild(monitored_stage);
    }

    if (!monitored_stage_ptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("SetupMTCPlanToPose"),
                     "Monitored stage '%s' not found in task", monitored_stage.c_str());
        return BT::NodeStatus::FAILURE;
    }

    generate_pose->setPose(target_pose);
    generate_pose->setMonitoredStage(monitored_stage_ptr);

    // // ---------------- Compute IK ----------------
    auto compute_ik = std::make_unique<stages::ComputeIK>("compute ik", std::move(generate_pose));

    compute_ik->setGroup(group);
    compute_ik->setIKFrame(ik_frame);
    compute_ik->setMaxIKSolutions(max_iterations);
    compute_ik->properties().set("link_padding", link_padding);
    compute_ik->properties().set("trajectory_sampling_rate", trajectory_sampling_rate);
    compute_ik->properties().configureInitFrom(Stage::INTERFACE,
                                                        { "target_pose" });

    if (keep_orientation && !orientation_links.empty())
    {
        compute_ik->properties().set("keep_orientation", true);
        compute_ik->properties().set("keep_orientation_links", orientation_links);
        compute_ik->properties().set("orientation_tolerance", orientation_tol);
    }

    task->add(std::move(compute_ik));

    // ---------------- Output ----------------
    setOutput("task", task);
    return BT::NodeStatus::SUCCESS;
}
} // namespace SetupMTCPlanToPose
