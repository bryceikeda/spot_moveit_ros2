#include "spot_behavior_tree/plugins/action/setup_mtc_connect_with_rrt.h"

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
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include "spot_behavior_tree/string_vector.h"
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

using namespace moveit::task_constructor;

BT_REGISTER_NODES(factory)
{
    SetupMTCConnectWithRRT::RegisterNodes(factory);
}

namespace SetupMTCConnectWithRRT
{
SetupMTCConnectWithRRT::SetupMTCConnectWithRRT(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList SetupMTCConnectWithRRT::providedPorts()
{
    return {
        BT::InputPort<double>("acceleration_scale_factor"),
        BT::InputPort<bool>("keep_orientation"),
        BT::InputPort<std::vector<std::string>>("keep_orientation_link_names"),
        BT::InputPort<double>("keep_orientation_tolerance"),
        BT::InputPort<int>("max_iterations"),
        BT::InputPort<double>("link_padding"),
        BT::InputPort<std::string>("planning_group_name"),
        BT::InputPort<unsigned int>("trajectory_sampling_rate"),
        BT::InputPort<double>("velocity_scale_factor"),
        BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
    };
}

BT::NodeStatus SetupMTCConnectWithRRT::tick()
{
    // ---------------- Inputs ----------------
    std::string group, ik_frame, monitored_stage;
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
    getInput("max_iterations", max_iterations);
    getInput("keep_orientation", keep_orientation);
    getInput("keep_orientation_tolerance", orientation_tol);
    getInput("acceleration_scale_factor", acceleration_scale_factor);
    getInput("velocity_scale_factor", velocity_scale_factor);
    getInput("keep_orientation_link_names", orientation_links);
    getInput("link_padding", link_padding);
    getInput("trajectory_sampling_rate", trajectory_sampling_rate);
    getInput("task", task);

    rclcpp::Node::SharedPtr node_ptr;
    if (!config().blackboard->rootBlackboard()->get("node", node_ptr))
        throw BT::RuntimeError("SetupMTCConnectWithRRT -> Missing rclcpp::Node::SharedPtr 'node' in blackboard");

    // ---------------- Generate Pose ----------------
    auto planner = std::make_shared<solvers::PipelinePlanner>(node_ptr, "ompl");

    planner->setPlannerId("RRTConnect"); 

    planner->setMaxVelocityScalingFactor(velocity_scale_factor);
    planner->setMaxAccelerationScalingFactor(acceleration_scale_factor);
    planner->setProperty("max_sampling_attempts", max_iterations);

    auto connect = std::make_unique<stages::Connect>(
        "connect",
        stages::Connect::GroupPlannerVector{{group, planner}}
    );
    connect->setProperty("trajectory_sampling_rate", trajectory_sampling_rate);
    connect->setProperty("link_padding", link_padding);
    
    if (keep_orientation && !orientation_links.empty())
    {
        moveit_msgs::msg::Constraints constraints;

        for (const auto& link : orientation_links)
        {
            moveit_msgs::msg::OrientationConstraint oc;
            oc.link_name = link;
            oc.header.frame_id = "world";

            // lock X & Y, allow Z rotation
            oc.absolute_x_axis_tolerance = orientation_tol;
            oc.absolute_y_axis_tolerance = orientation_tol;
            oc.absolute_z_axis_tolerance = M_PI;

            oc.weight = 1.0;

            constraints.orientation_constraints.push_back(oc);
        }

        connect->properties().set("path_constraints", constraints);
    }

    
    task->add(std::move(connect));



    // ---------------- Output ----------------
    setOutput("task", task);
    return BT::NodeStatus::SUCCESS;
}
} // namespace SetupMTCConnectWithRRT
