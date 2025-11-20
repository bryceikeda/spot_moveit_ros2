#include "spot_behavior_tree/plugins/action/create_stamped_pose.h"
#include <rclcpp/rclcpp.hpp>
#include "spot_behavior_tree/position_xyz.h"
#include "spot_behavior_tree/orientation_xyzw.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

BT_REGISTER_NODES(factory)
{
  CreateStampedPose::RegisterNodes(factory);
}

namespace CreateStampedPose{
CreateStampedPose::CreateStampedPose(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus CreateStampedPose::tick()
{
    std::string reference_frame;
    if (!getInput<std::string>("reference_frame", reference_frame))
        throw BT::RuntimeError("CreateStampedPose -> Missing required input [reference_frame]");

    PositionXYZ position_xyz;
    if (!getInput<PositionXYZ>("position_xyz", position_xyz))
        throw BT::RuntimeError("CreateStampedPose -> Missing required input [position_xyz]");

    OrientationXYZW orientation_xyzw;
    if (!getInput<OrientationXYZW>("orientation_xyzw", orientation_xyzw))
        throw BT::RuntimeError("CreateStampedPose -> Missing required input [orientation_xyzw]");

    // --- Set the target pose ---
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = reference_frame; 
    stamped_pose.pose.position.x = position_xyz.x;
    stamped_pose.pose.position.y = position_xyz.y;
    stamped_pose.pose.position.z = position_xyz.z;
    stamped_pose.pose.orientation.x = orientation_xyzw.x;
    stamped_pose.pose.orientation.y = orientation_xyzw.y;
    stamped_pose.pose.orientation.z = orientation_xyzw.z;
    stamped_pose.pose.orientation.w = orientation_xyzw.w;

    setOutput("stamped_pose", stamped_pose);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList CreateStampedPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("reference_frame"),
        BT::InputPort<PositionXYZ>("position_xyz"),
        BT::InputPort<OrientationXYZW>("orientation_xyzw"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("stamped_pose")
    };
}
}
