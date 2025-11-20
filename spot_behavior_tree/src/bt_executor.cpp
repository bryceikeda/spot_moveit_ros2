// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>



// Example that shows how to customize TreeExecutionServer.
//
// Here, we:
// - add an extra logger, BT::StdCoutLogger
// - add a subscriber that will continuously update a variable in the global blackboard


class MyActionServer : public BT::TreeExecutionServer
{
public:
  MyActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options)
  {
    // here we assume that the battery voltage is published as a std_msgs::msg::Float32
    // sub_ = node()->create_subscription<std_msgs::msg::Float32>(
    //     "battery_level", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
    //       // Update the global blackboard
    //       globalBlackboard()->set("battery_level", msg->data);
    //     });
    // Note that the callback above and the execution of the tree accessing the
    // global blackboard happen in two different threads.
    // The former runs in the MultiThreadedExecutor, while the latter in the thread created
    // by TreeExecutionServer. But this is OK because the blackboard is thread-safe.
    // --- Initialize MoveIt Context ---
    RCLCPP_INFO(node()->get_logger(), "Initializing MoveIt interfaces...");

    // Create MoveGroupInterface, PlanningSceneInterface, and VisualTools
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node(), PLANNING_GROUP);
    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        node(), "body", "moveit_demo", move_group_->getRobotModel());

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    // // --- Store everything in the global blackboard ---
    globalBlackboard()->set("move_group", move_group_);
    globalBlackboard()->set("planning_scene", planning_scene_);
    globalBlackboard()->set("visual_tools", visual_tools_);
    globalBlackboard()->set("PLANNING_GROUP", "arm");

    RCLCPP_INFO(node()->get_logger(), "MoveIt context stored in global blackboard.");
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                      bool was_cancelled) override
  {
    // NOT really needed, even if logger_cout_ may contain a dangling pointer of the tree
    // at this point
    logger_cout_.reset();
    return std::nullopt;
  }

private:
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::string PLANNING_GROUP = "arm";

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<MyActionServer>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}


