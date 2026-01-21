#include "spot_behavior_tree/plugins/action/transform_pose.h"
#include <rclcpp/rclcpp.hpp>
#include "spot_behavior_tree/position_xyz.h"
#include "spot_behavior_tree/orientation_xyzw.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

BT_REGISTER_NODES(factory)
{
  TransformPose::RegisterNodes(factory);
}

namespace TransformPose{
TransformPose::TransformPose(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus TransformPose::tick()
{
    geometry_msgs::msg::PoseStamped input_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("input_pose", input_pose))
        throw BT::RuntimeError("TransformPose -> Missing required input [input_pose]");

    PositionXYZ position_xyz;
    if (!getInput<PositionXYZ>("position_xyz", position_xyz))
        throw BT::RuntimeError("TransformPose -> Missing required input [position_xyz]");

    OrientationXYZW orientation_xyzw;
    if (!getInput<OrientationXYZW>("orientation_xyzw", orientation_xyzw))
        throw BT::RuntimeError("TransformPose -> Missing required input [orientation_xyzw]");

    // --- Set the target pose ---
    tf2::Transform T_in;
    tf2::fromMsg(input_pose.pose, T_in);

    // Build offset transform
    tf2::Transform T_offset;
    tf2::Quaternion q;
    q.setValue(orientation_xyzw.x,
               orientation_xyzw.y,
               orientation_xyzw.z,
               orientation_xyzw.w);

    T_offset.setOrigin(tf2::Vector3(position_xyz.x,
                                   position_xyz.y,
                                   position_xyz.z));
    T_offset.setRotation(q);

    // Compose transforms
    tf2::Transform T_out = T_in * T_offset.inverse();

    // Convert back to PoseStamped
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header = input_pose.header;  // preserve frame + timestamp
    output_pose.pose.position.x = T_out.getOrigin().x();
    output_pose.pose.position.y = T_out.getOrigin().y();
    output_pose.pose.position.z = T_out.getOrigin().z();
    output_pose.pose.orientation = tf2::toMsg(T_out.getRotation());
    setOutput("output_pose", output_pose);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList TransformPose::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose"),
        BT::InputPort<PositionXYZ>("position_xyz"),
        BT::InputPort<OrientationXYZW>("orientation_xyzw"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose")
    };
}
}
