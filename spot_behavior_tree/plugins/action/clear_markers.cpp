#include "spot_behavior_tree/plugins/action/clear_markers.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_visual_tools/moveit_visual_tools.h>


BT_REGISTER_NODES(factory)
{
  ClearMarkers::RegisterNodes(factory);
}

namespace ClearMarkers{
    ClearMarkers::ClearMarkers(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus ClearMarkers::tick()
    {
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
        if (!getInput<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools", visual_tools_))
            throw BT::RuntimeError("ClearMarkers -> Missing required input [visual_tools_]");

        visual_tools_->deleteAllMarkers();   // Clear any existing markers in RViz


        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList ClearMarkers::providedPorts()
    {
        return {
            BT::InputPort<moveit_visual_tools::MoveItVisualToolsPtr>("visual_tools"),
        };
    }
}
