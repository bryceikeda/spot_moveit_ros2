#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/attached_collision_object.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Logger for console output
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_joint_space_goal");

// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
int main(int argc, char* argv[])
{
    // --- ROS 2 Initialization ---
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_to_joint_space_goal", node_options);

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
    visual_tools.publishText(text_pose, "move_to_joint_space_goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to plan to a joint-space goal.");

    // --- Step 1: Retrieve the current joint positions for modification ---
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // --- Step 2: Define Joint-Space Goal ---
    // Modify joint values to define the target pose in joint space
    joint_group_positions[0] = 1.36136;   // arm_sh0
    joint_group_positions[1] = -1.90241;  // arm_sh1
    joint_group_positions[2] = 2.0944;    // arm_el0
    joint_group_positions[3] = -1.6057;   // arm_el1
    joint_group_positions[4] = 1.36136;   // arm_wr0
    joint_group_positions[5] = 1.78024;   // arm_wr1

    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits; planning will clamp values.");
    }

    // Reduce motion speed for safety: 5% of maximum velocity and acceleration
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    // --- Step 3: Plan Motion to Joint-Space Goal ---
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Planning to joint-space goal %s", success ? "succeeded" : "failed");

    // --- Step 4: Visualize the Planned Trajectory ---
    RCLCPP_INFO(LOGGER, "Visualizing planned trajectory...");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to execute the plan.");

    // --- Step 5: Execute the Planned Motion ---
    move_group.execute(my_plan);

    rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = once every 5 seconds
    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
