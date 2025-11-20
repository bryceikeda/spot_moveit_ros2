#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/attached_collision_object.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Logger for console output
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_object_over_obstacle");

// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
int main(int argc, char* argv[])
{
    // --- ROS 2 Initialization ---
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_object_over_obstacle", node_options);
    
    // Create a SingleThreadedExecutor for monitoring robot state in the background.
    // This ensures the MoveGroupInterface can access up-to-date state information.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // --- MoveIt Interface Setup ---
    // MoveIt organizes joints into named "planning groups" for motion planning.
    // We'll use two groups: one for the arm and one for the gripper.
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string GRIPPER_PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface arm_move_group(move_group_node, ARM_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_move_group(move_group_node, GRIPPER_PLANNING_GROUP);

    // Use a raw pointer to the arm’s joint model group for better performance
    // when visualizing planned trajectories
    const moveit::core::JointModelGroup* joint_model_group =
        arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);

    // The PlanningSceneInterface allows us to modify the environment—
    // adding or removing collision objects that affect motion planning.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // --- Visualization Setup ---
    // MoveItVisualTools provides helper functions for visualizing poses,
    // trajectories, and markers in RViz.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(
        move_group_node, "body", "demo", gripper_move_group.getRobotModel());

    visual_tools.deleteAllMarkers();   // Clear any existing markers in RViz
    visual_tools.loadRemoteControl();  // Enable "Next" button control from RViz GUI

    // Display the demo title in RViz
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "move_object_over_obstacle", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to move to the start pose.");

    // --- Step 1: Open the gripper before moving ---
    // The "open" target is set in the robot's SRDF
    RCLCPP_INFO(LOGGER, "Opening gripper...");
    gripper_move_group.setNamedTarget("open");
    gripper_move_group.move();

    // --- Step 2: Move the arm to its initial "start" pose ---
    RCLCPP_INFO(LOGGER, "Moving to start pose...");
    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.707;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 0.707;
    start_pose.position.x = 0.6;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.25;

    arm_move_group.setPoseTarget(start_pose);
    visual_tools.publishAxisLabeled(start_pose, "start_pose");
    visual_tools.trigger();
    arm_move_group.move();

    visual_tools.prompt("Press 'next' in RViz to add objects to the scene.");

    // --- Step 3: Add a cylindrical object to the scene and attach it to the end effector ---
    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions = {0.20, 0.04};  // height, radius

    object_to_attach.header.frame_id = arm_move_group.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.y = 0.7071;
    grab_pose.orientation.w = 0.7071;
    grab_pose.position.x = 0.275;
    grab_pose.position.z = 0.01;

    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;

    // applyCollisionObject adds a single object at a time synchronously
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // --- Step 4: Add a static obstacle (box) to the world ---
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "box1";
    collision_object.header.frame_id = arm_move_group.getPlanningFrame();

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.1, 1.5, 0.5};  // width, depth, height

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.825;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.1;  

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // addCollisionObjects can add multiple objects as a vector
    planning_scene_interface.addCollisionObjects({collision_object});
    RCLCPP_INFO(LOGGER, "Added collision objects to the world.");

    visual_tools.prompt("Press 'next' in RViz once the objects appear.");

    // --- Step 5: Attach the cylinder to the gripper to simulate a grasp ---
    RCLCPP_INFO(LOGGER, "Attaching object to the robot...");
    std::vector<std::string> touch_links = {"arm_link_fngr", "arm_link_wr1"};
    arm_move_group.attachObject(object_to_attach.id, "arm_link_wr0", touch_links);

    RCLCPP_INFO(LOGGER, "Closing gripper to grasp object...");
    gripper_move_group.setNamedTarget("close");
    gripper_move_group.move();

    visual_tools.prompt("Press 'next' in RViz once object is attached.");

    // --- Step 6: Plan motion to the target pose ---
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.707;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.707;
    target_pose.position.x = 0.975;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.25;

    arm_move_group.setPoseTarget(target_pose);

    // --- Step 7: Plan Motion to Target Pose ---
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Planning to target pose %s", success ? "succeeded" : "failed");

    // --- Step 8: Visualize Planned Trajectory ---
    RCLCPP_INFO(LOGGER, "Visualizing planned trajectory...");
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in RViz to execute the plan.");

    // --- Step 8: Execute the Planned Motion ---
    arm_move_group.execute(my_plan);

    visual_tools.prompt("Press 'next' in RViz to release the object and stow the arm.");

    // --- Step 9: Open gripper to release the object ---
    RCLCPP_INFO(LOGGER, "Opening gripper to release object...");
    gripper_move_group.setNamedTarget("open");
    gripper_move_group.move();

    // --- Step 10: Detach and remove the object from the robot ---
    RCLCPP_INFO(LOGGER, "Detaching and removing object from planning scene...");
    arm_move_group.detachObject(object_to_attach.id);

    // --- Step 11: Move arm back to stow pose ---
    RCLCPP_INFO(LOGGER, "Returning arm to stow pose...");
    arm_move_group.setNamedTarget("stow");  
    arm_move_group.move();

    // --- Step 12: Closing gripper ---
    RCLCPP_INFO(LOGGER, "Closing gripper...");
    gripper_move_group.setNamedTarget("close");  
    gripper_move_group.move();

    rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = once every 5 seconds
    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
