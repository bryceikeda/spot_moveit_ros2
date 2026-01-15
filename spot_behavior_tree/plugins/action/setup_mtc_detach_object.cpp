#include "spot_behavior_tree/plugins/action/setup_mtc_detach_object.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

BT_REGISTER_NODES(factory)
{
  SetupMTCDetachObject::RegisterNodes(factory);
}

namespace SetupMTCDetachObject{
    SetupMTCDetachObject::SetupMTCDetachObject(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus SetupMTCDetachObject::tick()
    {
        // ---- Inputs ----
        std::string frame_id, object_name; 
        std::shared_ptr<moveit::task_constructor::Task> task;
        if (!getInput("task", task))
        {
            throw BT::RuntimeError("SetupMTCDetachObject: Missing required input [task]");
        }

        getInput("frame_id", frame_id);
        getInput("object_name", object_name);

        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object_name, frame_id);
        task->add(std::move(stage));

        // ---- Output ----
        setOutput("task", task);

        return BT::NodeStatus::SUCCESS;
    }


    BT::PortsList SetupMTCDetachObject::providedPorts()
    {
        return {
            BT::InputPort<std::string>("frame_id"),
            BT::InputPort<std::string>("object_name"),
            BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>("task"),
        };
    }
}