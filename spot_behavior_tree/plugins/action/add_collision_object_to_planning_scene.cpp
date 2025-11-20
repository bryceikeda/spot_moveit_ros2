#include "spot_behavior_tree/plugins/action/add_collision_object_to_planning_scene.h"
#include <rclcpp/rclcpp.hpp>

bool AddCollisionObjectToPlanningScene::setMessage(moveit_msgs::msg::PlanningScene& msg)
{
    // Get the CollisionObject from the blackboard
    moveit_msgs::msg::CollisionObject collision_object;
    if (!getInput("collision_object", collision_object))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AddCollisionObjectToPlanningScene"),
                     "Missing input [collision_object]");
        return false;
    }

    // Ensure operation is ADD
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Fill PlanningScene message
    msg.is_diff = true;  // we only want to modify the current scene
    msg.world.collision_objects.clear(); // clear any previous objects in this message
    msg.world.collision_objects.push_back(collision_object);

    // Optional: set the header from the collision_object if available
    msg.robot_state.is_diff = true;  // leave robot state as is

    RCLCPP_INFO(rclcpp::get_logger("AddCollisionObjectToPlanningScene"),
                "Publishing collision object: %s", collision_object.id.c_str());

    return true;
}

// Register the node as a plugin
CreateRosNodePlugin(AddCollisionObjectToPlanningScene, "AddCollisionObjectToPlanningScene");