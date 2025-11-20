#include "spot_behavior_tree/plugins/action/create_collision_object_from_solid_primitive.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "shape_msgs/msg/solid_primitive.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

BT_REGISTER_NODES(factory)
{
  CreateCollisionObjectFromSolidPrimitive::RegisterNodes(factory);
}

namespace CreateCollisionObjectFromSolidPrimitive
{

CreateCollisionObjectFromSolidPrimitive::CreateCollisionObjectFromSolidPrimitive(const std::string& name,
                                                 const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus CreateCollisionObjectFromSolidPrimitive::tick()
{
    shape_msgs::msg::SolidPrimitive solid_primitive;
    if (!getInput("solid_primitive", solid_primitive))
        throw BT::RuntimeError("CreateCollisionObjectFromSolidPrimitive -> Missing required input [solid_primitive]");

    geometry_msgs::msg::PoseStamped pose; 
    if (!getInput("pose", pose))
        throw BT::RuntimeError("CreateCollisionObjectFromSolidPrimitive -> Missing required input [pose]");
    
    std::string object_id; 
    if (!getInput("object_id", object_id))
        throw BT::RuntimeError("CreateCollisionObjectFromSolidPrimitive -> Missing required input [object_id]");

    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.pose = pose.pose;
    collision_object.header = pose.header; 
    collision_object.id = object_id;
    collision_object.primitives.push_back(solid_primitive);

    setOutput("collision_object", collision_object);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList CreateCollisionObjectFromSolidPrimitive::providedPorts()
{
    return {
        BT::InputPort<shape_msgs::msg::SolidPrimitive>("solid_primitive"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
        BT::InputPort<std::string>("object_id"),
        BT::OutputPort<moveit_msgs::msg::CollisionObject>("collision_object")
    };
}

} 