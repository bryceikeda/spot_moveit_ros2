#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/attached_collision_object.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Logger for node output
static const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_movement_to_waypoints");

// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
int main(int argc, char* argv[])
{
    // --- ROS 2 Initialization ---
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("cartesian_movement_to_waypoints", node_options);

    // Create a SingleThreadedExecutor for monitoring robot state in the background.
    // This ensures the MoveGroupInterface can access up-to-date state information.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // --- MoveIt Interface Setup ---
    // MoveIt organizes joints into named "planning groups" for motion planning.
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

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
    visual_tools.publishText(text_pose, "cartesian_movement_to_waypoints", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to move to the start pose.");

    // --- Step 1: Move to Start Pose ---
    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.55;
    start_pose.position.y = -0.05;
    start_pose.position.z = 0.8;

    move_group.setPoseTarget(start_pose);
    move_group.move();

    visual_tools.prompt("Press 'next' in RViz to plan a Cartesian path through waypoints.");

    // --- Step 2: Define Waypoints for a Cartesian Path ---
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);  // Start from the initial pose

    geometry_msgs::msg::Pose target_pose = start_pose;

    // Move the end effector down
    target_pose.position.z -= 0.2;
    waypoints.push_back(target_pose);

    // Move right
    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);

    // Move up and left
    target_pose.position.z += 0.2;
    target_pose.position.y += 0.2;
    target_pose.position.x -= 0.2;
    waypoints.push_back(target_pose);

    // --- Step 3: Compute Cartesian Path ---
    // Compute a straight-line motion in Cartesian space interpolated at 1 cm resolution.
    const double eef_step = 0.01;      // Interpolation step (1 cm)
    const double jump_threshold = 0.0; // Disable jump detection
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Computed Cartesian path (%.2f%% achieved)", fraction * 100.0);

    // --- Step 4: Visualize the Cartesian Path ---
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);

    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in RViz to execute the Cartesian trajectory.");

    // --- Step 5: Execute the Trajectory ---
    // Cartesian trajectories are often executed slowly to ensure safe motion near objects.
    // Currently, Cartesian motion speed must be adjusted by manually timing the trajectory.
    move_group.execute(trajectory);

    rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = once every 5 seconds
    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    
    return 0;
}
