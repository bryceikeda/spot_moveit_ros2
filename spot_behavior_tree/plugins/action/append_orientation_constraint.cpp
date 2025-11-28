#include "spot_behavior_tree/plugins/action/append_orientation_constraint.h"
#include "moveit_msgs/msg/orientation_constraint.h"
#include "moveit_msgs/msg/constraints.h"
#include "spot_behavior_tree/orientation_xyzw.h"
#include "spot_behavior_tree/axis_tolerance_xyz.h"

BT_REGISTER_NODES(factory)
{
  AppendOrientationConstraint::RegisterNodes(factory);
}

namespace AppendOrientationConstraint{
AppendOrientationConstraint::AppendOrientationConstraint(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus AppendOrientationConstraint::tick()
{
    moveit_msgs::msg::Constraints constraints;
    if (!getInput<moveit_msgs::msg::Constraints>("constraints", constraints))
        throw BT::RuntimeError("AppendOrientationConstraint -> Missing required input [constraints]");

    AxisToleranceXYZ tolerance_xyz;
    if (!getInput<AxisToleranceXYZ>("tolerance_xyz", tolerance_xyz))
        throw BT::RuntimeError("AppendOrientationConstraint -> Missing required input [tolerance_xyz]");
    
    OrientationXYZW orientation_xyzw;
    if (!getInput<OrientationXYZW>("orientation_xyzw", orientation_xyzw))
        throw BT::RuntimeError("AppendOrientationConstraint -> Missing required input [orientation_xyzw]");
    
    std::string link_name;
    if (!getInput<std::string>("link_name", link_name))
        throw BT::RuntimeError("AppendOrientationConstraint -> Missing required input [link_name]");
            
    std::string frame_id;
    if (!getInput<std::string>("frame_id", frame_id))
        throw BT::RuntimeError("AppendOrientationConstraint -> Missing required input [frame_id]");
    
    double weight;
    if (!getInput<double>("weight", weight))
        throw BT::RuntimeError("AppendOrientationConstraint -> Missing required input [weight]");
    
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = link_name;
    ocm.header.frame_id = frame_id;     
    ocm.orientation.x = orientation_xyzw.x;           
    ocm.orientation.y = orientation_xyzw.y; 
    ocm.orientation.z = orientation_xyzw.z; 
    ocm.orientation.w = orientation_xyzw.w; 
    ocm.absolute_x_axis_tolerance = tolerance_xyz.x;
    ocm.absolute_y_axis_tolerance = tolerance_xyz.y;
    ocm.absolute_z_axis_tolerance = tolerance_xyz.z;
    ocm.weight = weight;

    constraints.orientation_constraints.push_back(ocm);

    setOutput("constraints", constraints);
    return BT::NodeStatus::SUCCESS;

}

BT::PortsList AppendOrientationConstraint::providedPorts()
{
    return {
        BT::InputPort<AxisToleranceXYZ>("tolerance_xyz"),
        BT::InputPort<OrientationXYZW>("orientation_xyzw"),
        BT::InputPort<std::string>("link_name"),
        BT::InputPort<std::string>("frame_id"),
        BT::InputPort<double>("weight"),
        BT::BidirectionalPort<moveit_msgs::msg::Constraints>("constraints"),
    };
}
}
