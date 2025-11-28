#include "spot_behavior_tree/plugins/action/add_ros_node_to_blackboard.h"
#include <rclcpp/rclcpp.hpp>

bool AddRosNodeToBlackboard::setMessage(std_msgs::msg::Empty& msg)
{
    config().blackboard->rootBlackboard()->set("node", node);
    return true;
}

// Register the node as a plugin
CreateRosNodePlugin(AddRosNodeToBlackboard, "AddRosNodeToBlackboard");