#include "spot_behavior_tree/plugins/action/is_collision_object_in_planning_scene.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_msgs/msg/planning_scene.hpp>


BT_REGISTER_NODES(factory)
{
  IsCollisionObjectInPlanningScene::RegisterNodes(factory);
}

namespace IsCollisionObjectInPlanningScene{
    IsCollisionObjectInPlanningScene::IsCollisionObjectInPlanningScene(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus IsCollisionObjectInPlanningScene::tick()
    {
        moveit_msgs::msg::PlanningScene planning_scene;
        if (!getInput<moveit_msgs::msg::PlanningScene>("planning_scene", planning_scene))
            throw BT::RuntimeError("IsCollisionObjectInPlanningScene -> Missing required input [planning_scene]");
        
        std::string collision_object_name; 
        if (!getInput<std::string>("collision_object_name", collision_object_name))
            throw BT::RuntimeError("IsCollisionObjectInPlanningScene -> Missing required input [collision_object_name]");

        for (const auto& obj : planning_scene.world.collision_objects)
        {
            if (obj.id == collision_object_name)
            {
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::FAILURE;
    }

    BT::PortsList IsCollisionObjectInPlanningScene::providedPorts()
    {
        return {
            BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene"),
            BT::InputPort<std::string>("collision_object_name")
        };
    }
}
