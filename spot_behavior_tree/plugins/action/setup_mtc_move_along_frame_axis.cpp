#include "spot_behavior_tree/plugins/action/setup_mtc_move_along_frame_axis.h"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include "spot_behavior_tree/string_vector.h"
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>

using namespace moveit::task_constructor;

BT_REGISTER_NODES(factory)
{
    SetupMTCMoveAlongFrameAxis::RegisterNodes(factory);
}

namespace SetupMTCMoveAlongFrameAxis
{
SetupMTCMoveAlongFrameAxis::SetupMTCMoveAlongFrameAxis(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList SetupMTCMoveAlongFrameAxis::providedPorts()
{
    return {
        BT::InputPort<double>("acceleration_scale_factor"),
        BT::InputPort<std::string>("planning_group_name"),
        BT::InputPort<double>("max_distance"),
        BT::InputPort<double>("min_distance"),
        BT::InputPort<double>("axis_z"),
        BT::InputPort<double>("axis_y"),
        BT::InputPort<double>("axis_x"),
        BT::InputPort<bool>("ignore_environment_collisions"),
        BT::InputPort<double>("velocity_scale_factor"),
        BT::InputPort<std::string>("hand_frame"),
        BT::InputPort<std::string>("axis_frame"),
        BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
    };
}

BT::NodeStatus SetupMTCMoveAlongFrameAxis::tick()
{
    // ---------------- Inputs ----------------
    std::string axis_frame, group, hand_frame;
    double max_distance = 1.0;
    double min_distance = 0.1;
    double axis_z = 0.0;
    double axis_y = 0.0;
    double axis_x = -1;
    bool ignore_environment_collisions = true;
    double velocity_scale_factor = 1.0;
    double acceleration_scale_factor = 1.0;

    std::shared_ptr<moveit::task_constructor::Task> task;

    getInput("acceleration_scale_factor", acceleration_scale_factor);
    getInput("planning_group_name", group);
    getInput("max_distance", max_distance);
    getInput("min_distance", min_distance);
    getInput("axis_z", axis_z);
    getInput("axis_y", axis_y);
    getInput("axis_x", axis_x);
    getInput("velocity_scale_factor", velocity_scale_factor);
    getInput("axis_frame", axis_frame);
    getInput("hand_frame", hand_frame);
    getInput("ignore_environment_collisions", ignore_environment_collisions);
    getInput("task", task);

    rclcpp::Node::SharedPtr node_ptr;
    if (!config().blackboard->rootBlackboard()->get("node", node_ptr))
        throw BT::RuntimeError("SetupMTCMoveAlongFrameAxis -> Missing rclcpp::Node::SharedPtr 'node' in blackboard");

	// Cartesian planner
	cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(velocity_scale_factor);
	cartesian_planner->setMaxAccelerationScalingFactor(acceleration_scale_factor);
	cartesian_planner->setStepSize(.01);

    // ---------------- Move Relative ----------------
    // Move the eef link forward along an axis by an amount within the given min-max range
    auto stage = std::make_unique<stages::MoveRelative>("move along frame axis", cartesian_planner);
    stage->properties().set("marker_ns", "move_along_frame_axis");
    stage->properties().set("link", hand_frame);  // link to perform IK for
    stage->properties().set("group", group);  

    geometry_msgs::msg::PoseStamped ik_frame;
    ik_frame.header.frame_id = hand_frame;
    ik_frame.pose.orientation.w = 1;

    stage->properties().set("ik_frame", ik_frame);
    stage->setMinMaxDistance(min_distance, max_distance);

    // Set hand direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = axis_frame;
    vec.vector.z = axis_z;
    vec.vector.y = axis_y;
    vec.vector.x = axis_x;
    stage->setDirection(vec);
    task->insert(std::move(stage));

    // ---------------- Output ----------------
    setOutput("task", task);
    return BT::NodeStatus::SUCCESS;
}
} // namespace SetupMTCMoveAlongFrameAxis
