#pragma once

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

using namespace BT;

class AddCollisionObjectToPlanningScene
  : public BT::RosTopicPubNode<moveit_msgs::msg::PlanningScene>
{
public:
    AddCollisionObjectToPlanningScene(const std::string& name,
                                      const BT::NodeConfig& config,
                                      const RosNodeParams& params)
        : RosTopicPubNode<moveit_msgs::msg::PlanningScene>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<moveit_msgs::msg::CollisionObject>("collision_object")
        });
    }

    bool setMessage(moveit_msgs::msg::PlanningScene& msg) override;
};
