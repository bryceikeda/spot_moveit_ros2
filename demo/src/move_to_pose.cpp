#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/attached_collision_object.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Logger for console output
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_pose");

// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
int main(int argc, char* argv[])
{
    // --- ROS 2 Initialization ---
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_to_pose", node_options);

    // Create a SingleThreadedExecutor for monitoring robot state in the background.
    // This ensures the MoveGroupInterface can access up-to-date state information.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // --- MoveIt Interface Setup ---
    // MoveIt organizes joints into named "planning groups" for motion planning.
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // Use a raw pointer to the arm’s joint model group for better performance
    // when visualizing planned trajectories
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // --- Visualization Setup ---
    // MoveItVisualTools provides helper functions for visualizing poses,
    // trajectories, and markers in RViz.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(
        move_group_node, "body", "demo", move_group.getRobotModel());

    visual_tools.deleteAllMarkers();   // Clear any existing markers in RViz
    visual_tools.loadRemoteControl();  // Enable "Next" button control from RViz GUI

    // Display the demo title in RViz
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "move_to_pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to plan to a target pose.");

    // --- Step 1: Define Target Pose ---
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;

    move_group.setPoseTarget(target_pose);

    // --- Step 2: Plan Motion to Target Pose ---
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Planning to target pose %s", success ? "succeeded" : "failed");

    // --- Step 3: Visualize Planned Trajectory ---
    RCLCPP_INFO(LOGGER, "Visualizing planned trajectory...");
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to execute the plan.");

    // --- Step 4: Execute the Planned Motion ---
    move_group.execute(my_plan);

    rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = once every 5 seconds
    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
