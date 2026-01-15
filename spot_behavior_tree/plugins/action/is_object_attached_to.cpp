#include "spot_behavior_tree/plugins/action/is_object_attached_to.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_msgs/msg/planning_scene.hpp>


BT_REGISTER_NODES(factory)
{
  IsObjectAttachedTo::RegisterNodes(factory);
}

namespace IsObjectAttachedTo{
    IsObjectAttachedTo::IsObjectAttachedTo(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus IsObjectAttachedTo::tick()
    {
        moveit_msgs::msg::PlanningScene planning_scene;
        if (!getInput<moveit_msgs::msg::PlanningScene>("planning_scene", planning_scene))
            throw BT::RuntimeError("IsObjectAttachedTo -> Missing required input [planning_scene]");
        
        std::string link_name; 
        if (!getInput<std::string>("link_name", link_name))
            throw BT::RuntimeError("IsObjectAttachedTo -> Missing required input [link_name]");

        std::string object_id; 
        if (!getInput<std::string>("object_id", object_id))
            throw BT::RuntimeError("IsObjectAttachedTo -> Missing required input [object_id]");


        for (const auto& obj : planning_scene.robot_state.attached_collision_objects)
        {
            if (obj.object.id == object_id && obj.link_name == link_name)
            {
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::FAILURE;
    }

    BT::PortsList IsObjectAttachedTo::providedPorts()
    {
        return {
            BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene"),
            BT::InputPort<std::string>("link_name"),
            BT::InputPort<std::string>("object_id")
        };
    }
}