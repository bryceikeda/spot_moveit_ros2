#include "spot_behavior_tree/plugins/action/setup_mtc_ignore_collisions_between_objects.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

BT_REGISTER_NODES(factory)
{
  SetupMTCIgnoreCollisionsBetweenObjects::RegisterNodes(factory);
}

namespace SetupMTCIgnoreCollisionsBetweenObjects{
    SetupMTCIgnoreCollisionsBetweenObjects::SetupMTCIgnoreCollisionsBetweenObjects(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupMTCIgnoreCollisionsBetweenObjects::tick()
    {
        // ---- Inputs ----
        std::vector<std::string> object_names; 
        bool allow_collisions;
        std::shared_ptr<moveit::task_constructor::Task> task;
        if (!getInput("task", task))
        {
            throw BT::RuntimeError("SetupMTCIgnoreCollisionsBetweenObjects: Missing required input [task]");
        }

        getInput("object_names", object_names);
        getInput("allow_collisions", allow_collisions);

        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("collision (group of objects)");
        stage->allowCollisions(object_names, object_names, allow_collisions);
        task->add(std::move(stage));

        // ---- Output ----
        setOutput("task", task);

        return BT::NodeStatus::SUCCESS;
    }


    BT::PortsList SetupMTCIgnoreCollisionsBetweenObjects::providedPorts()
    {
        return {
            BT::InputPort<bool>("allow_collisions"),
            BT::InputPort<std::vector<std::string>>("object_names"),
            BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
        };
    }
}