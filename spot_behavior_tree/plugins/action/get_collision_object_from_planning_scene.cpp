#include "spot_behavior_tree/plugins/action/get_collision_object_from_planning_scene.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

BT_REGISTER_NODES(factory)
{
  GetCollisionObjectFromPlanningScene::RegisterNodes(factory);
}

namespace GetCollisionObjectFromPlanningScene{
    GetCollisionObjectFromPlanningScene::GetCollisionObjectFromPlanningScene(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus GetCollisionObjectFromPlanningScene::tick()
    {
        moveit_msgs::msg::PlanningScene planning_scene;
        if (!getInput<moveit_msgs::msg::PlanningScene>("planning_scene", planning_scene))
            throw BT::RuntimeError("GetCollisionObjectFromPlanningScene -> Missing required input [planning_scene]");
        
        std::string object_id; 
        if (!getInput<std::string>("object_id", object_id))
            throw BT::RuntimeError("GetCollisionObjectFromPlanningScene -> Missing required input [object_id]");


        for (const auto& obj : planning_scene.world.collision_objects)
        {
            if (obj.id == object_id)
            {
                setOutput("collision_object", obj);
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::FAILURE;
    }

    BT::PortsList GetCollisionObjectFromPlanningScene::providedPorts()
    {
        return {
            BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene"),
            BT::InputPort<std::string>("object_id"),
            BT::OutputPort<moveit_msgs::msg::CollisionObject>("collision_object")
        };
    }
}