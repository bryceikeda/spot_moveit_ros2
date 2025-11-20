#include "spot_behavior_tree/plugins/action/add_to_vector.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<AddToVector<geometry_msgs::msg::PoseStamped>>("AddPoseStampedToVector");
}