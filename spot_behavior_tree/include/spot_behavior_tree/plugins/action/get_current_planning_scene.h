#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

using namespace BT;

class GetCurrentPlanningScene
  : public BT::RosServiceNode<moveit_msgs::srv::GetPlanningScene>
{
public:
    GetCurrentPlanningScene(const std::string& name,
                            const BT::NodeConfig& config,
                            const RosNodeParams& params)
        : RosServiceNode<moveit_msgs::srv::GetPlanningScene>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene_msg")
        });
    }

    bool setRequest(Request::SharedPtr& request) override;

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
