#include "spot_behavior_tree/plugins/action/setup_mtc_update_object_collision_rule.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

BT_REGISTER_NODES(factory)
{
  SetupMTCUpdateObjectCollisionRule::RegisterNodes(factory);
}

namespace SetupMTCUpdateObjectCollisionRule{
    SetupMTCUpdateObjectCollisionRule::SetupMTCUpdateObjectCollisionRule(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupMTCUpdateObjectCollisionRule::tick()
    {
        // ---- Inputs ----
        std::string object_or_group_name, object_name; 
        std::shared_ptr<moveit::task_constructor::Task> task;
        bool allow_collision;
        if (!getInput("task", task))
        {
            throw BT::RuntimeError("SetupMTCAttachObject: Missing required input [task]");
        }

        getInput("allow_collision", allow_collision);
        getInput("object_or_group_name", object_or_group_name);
        getInput("object_name", object_name);
        std::string stage_name =
            "collision (" + object_name + ", " + object_or_group_name + ")";
        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(stage_name);


        stage->allowCollisions({ object_name}, { object_or_group_name }, allow_collision);
        task->insert(std::move(stage));

        // ---- Output ----
        setOutput("task", task);

        return BT::NodeStatus::SUCCESS;
    }


    BT::PortsList SetupMTCUpdateObjectCollisionRule::providedPorts()
    {
        return {
            BT::InputPort<bool>("allow_collision"),
            BT::InputPort<std::string>("object_name"),
            BT::InputPort<std::string>("object_or_group_name"),
            BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
        };
    }
}