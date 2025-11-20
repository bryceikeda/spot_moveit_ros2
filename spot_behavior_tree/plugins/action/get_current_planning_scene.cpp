#include "spot_behavior_tree/plugins/action/get_current_planning_scene.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

bool GetCurrentPlanningScene::setRequest(Request::SharedPtr& request)
{
    // Request all components using the bitmask flags
    request->components.components = 
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
        moveit_msgs::msg::PlanningSceneComponents::OBJECT_COLORS;

    return true;
}

BT::NodeStatus GetCurrentPlanningScene::onResponseReceived(const Response::SharedPtr& response)
{
    std::cout << "onResponseReceived " << std::endl;

    if (response)
    {
        RCLCPP_INFO(logger(), "GetCurrentPlanningScene service succeeded.");

        // Set the output to pass the planning scene to other BT nodes
        setOutput("planning_scene_msg", response->scene);

        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(logger(), "GetCurrentPlanningScene service failed");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus GetCurrentPlanningScene::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "GetCurrentPlanningScene service call failed: %d", error);
    return BT::NodeStatus::FAILURE;
}

CreateRosNodePlugin(GetCurrentPlanningScene, "GetCurrentPlanningScene");
