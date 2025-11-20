#include "spot_behavior_tree/plugins/action/create_solid_primitive_box.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "shape_msgs/msg/solid_primitive.hpp"

BT_REGISTER_NODES(factory)
{
  CreateSolidPrimitiveBox::RegisterNodes(factory);
}

namespace CreateSolidPrimitiveBox
{

CreateSolidPrimitiveBox::CreateSolidPrimitiveBox(const std::string& name,
                                                 const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus CreateSolidPrimitiveBox::tick()
{
    std::vector<double> dimensions;
    if (!getInput("dimensions", dimensions))
        throw BT::RuntimeError("CreateSolidPrimitiveBox -> Missing required input [dimensions]");

    shape_msgs::msg::SolidPrimitive solid_primitive;
    solid_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

    solid_primitive.dimensions.assign(dimensions.begin(), dimensions.end());

    setOutput("solid_primitive", solid_primitive);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList CreateSolidPrimitiveBox::providedPorts()
{
    return {
        BT::InputPort<std::vector<double>>("dimensions"),
        BT::OutputPort<shape_msgs::msg::SolidPrimitive>("solid_primitive")
    };
}

} 