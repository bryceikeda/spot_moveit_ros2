#include "spot_behavior_tree/plugins/action/setup_mtc_ignore_graspable_collisions.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

BT_REGISTER_NODES(factory)
{
  SetupMTCIgnoreGraspableCollisions::RegisterNodes(factory);
}

namespace SetupMTCIgnoreGraspableCollisions{
    SetupMTCIgnoreGraspableCollisions::SetupMTCIgnoreGraspableCollisions(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupMTCIgnoreGraspableCollisions::tick()
    {
        // ---- Inputs ----
        std::string object_name, hand_name; 
        bool allow_collisions;
        std::shared_ptr<moveit::task_constructor::Task> task;
        if (!getInput("task", task))
        {
            throw BT::RuntimeError("SetupMTCIgnoreGraspableCollisions: Missing required input [task]");
        }

        getInput("object_name", object_name);
        getInput("hand_name", hand_name);
        getInput("allow_collisions", allow_collisions);

        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("collision (hand, object)");
        stage->allowCollisions(object_name, task->getRobotModel()->getJointModelGroup(hand_name)->getLinkModelNamesWithCollisionGeometry(), allow_collisions);
        task->add(std::move(stage));

        // ---- Output ----
        setOutput("task", task);

        return BT::NodeStatus::SUCCESS;
    }


    BT::PortsList SetupMTCIgnoreGraspableCollisions::providedPorts()
    {
        return {
            BT::InputPort<bool>("allow_collisions"),
            BT::InputPort<std::string>("object_name"),
            BT::InputPort<std::string>("hand_name"),
            BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
        };
    }
}