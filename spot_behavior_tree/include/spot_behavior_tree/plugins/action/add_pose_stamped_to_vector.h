#pragma once
#include <behaviortree_cpp/action_node.h>
#include <vector>
#include <behaviortree_cpp/bt_factory.h> 

namespace AddPoseStampedToVector
{
class AddPoseStampedToVector : public BT::SyncActionNode
{
    public:
      AddPoseStampedToVector(const std::string& name, const BT::NodeConfig& config);
      BT::NodeStatus tick() override;
      static BT::PortsList providedPorts();
};
    inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<AddPoseStampedToVector>("AddPoseStampedToVector");
    }
}
