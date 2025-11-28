#pragma once

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <std_msgs/msg/empty.hpp>

using namespace BT;

class AddRosNodeToBlackboard
  : public BT::RosTopicPubNode<std_msgs::msg::Empty>
{
public:
    AddRosNodeToBlackboard(const std::string& name,
                                      const BT::NodeConfig& config,
                                      const RosNodeParams& params)
        : RosTopicPubNode<std_msgs::msg::Empty>(name, config, params)
    {
      node = params.nh.lock();
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setMessage(std_msgs::msg::Empty& msg) override;

    private:
      rclcpp::Node::SharedPtr node;
};
