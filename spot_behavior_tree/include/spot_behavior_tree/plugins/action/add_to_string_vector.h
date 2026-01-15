#pragma once
#include <behaviortree_cpp/action_node.h>
#include <vector>
#include <behaviortree_cpp/bt_factory.h> 

namespace AddToStringVector
{
class AddToStringVector : public BT::SyncActionNode
{
    public:
      AddToStringVector(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};
    inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
    {
    factory.registerNodeType<AddToStringVector>("AddToStringVector");
    }
}