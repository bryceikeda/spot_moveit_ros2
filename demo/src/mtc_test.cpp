#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit/task_constructor/stage.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();
  std::shared_ptr<mtc::Task> task;
  void setupPlanningScene();

private:
  std::shared_ptr<mtc::Task> createTask();
  rclcpp::Node::SharedPtr node_;
  using ExecuteTaskSolution =
    moveit_task_constructor_msgs::action::ExecuteTaskSolution;

  rclcpp_action::Client<ExecuteTaskSolution>::SharedPtr exec_client_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  exec_client_ = rclcpp_action::create_client<ExecuteTaskSolution>(
      node_, "execute_task_solution");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "body";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.02};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.75;
  pose.position.z = 0.2;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  std::shared_ptr<mtc::Task> task_ = createTask();

  try
  {
    task_->init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e.what());
    return;
  }

  if (!task_->plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  // publishSolution expects a reference to SolutionBase, not a shared_ptr
  if (!task_->solutions().empty())
  {
    task_->introspection().publishSolution(*task_->solutions().front());
  }
  else
  {
    RCLCPP_WARN(LOGGER, "No solutions found to publish.");
  }
 	moveit_task_constructor_msgs::msg::Solution solution_msg;
	task_->solutions().front()->toMsg(solution_msg);
  RCLCPP_INFO(
        LOGGER,
        "MTC Solution: size=%zu id=%zu, sub_trajectory=%zu,traj 2=%zu",
        task_->solutions().size(),
        solution_msg.task_id.size(),
        solution_msg.sub_trajectory.size(),
        solution_msg.sub_trajectory[1].trajectory.joint_trajectory.points[0].positions.size()
    );


  if (!exec_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(LOGGER, "ExecuteTaskSolution action server not available");
    return;
  }

  ExecuteTaskSolution::Goal goal;
  goal.solution = solution_msg;

  auto send_goal_options =
      rclcpp_action::Client<ExecuteTaskSolution>::SendGoalOptions();

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<ExecuteTaskSolution>::WrappedResult& result)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR(LOGGER, "ExecuteTaskSolution failed");
      return;
    }

    if (result.result->error_code.val ==
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Task execution succeeded");
    }
    else
    {
      RCLCPP_ERROR(LOGGER,
                  "Task execution failed with MoveIt error code %d",
                  result.result->error_code.val);
    }
  };

  exec_client_->async_send_goal(goal, send_goal_options);
}

std::shared_ptr<mtc::Task> MTCTaskNode::createTask()
{
  task = std::make_shared<mtc::Task>();
  task->stages()->setName("demo task for mtc");
  task->loadRobotModel(node_);

  task->setProperty("timeout", 100.0);
  task->setProperty("trajectory_monitoring", false);
  auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");

  auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
  task->add(std::move(current_state));

  auto connect = std::make_unique<mtc::stages::Connect>(
      "connect",
      mtc::stages::Connect::GroupPlannerVector{{"arm", planner}}
  );
  task->add(std::move(connect));

  // ---------------- Generate Pose ----------------
  auto generate_pose = std::make_unique<mtc::stages::GeneratePose>("generate target pose");

  mtc::Stage* monitored_stage_ptr = task->stages()->findChild("current state");
  if (!monitored_stage_ptr)
  {
    RCLCPP_ERROR(LOGGER, "Monitored stage 'current state' not found in task");
  }

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "body";
  target_pose.pose.position.x = 0.28;
  target_pose.pose.position.y = -0.2;
  target_pose.pose.position.z = 0.5;
  target_pose.pose.orientation.w = 1.0;

  generate_pose->setPose(target_pose);
  if (monitored_stage_ptr)
    generate_pose->setMonitoredStage(monitored_stage_ptr);

  // ---------------- Compute IK ----------------
  auto compute_ik = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(generate_pose));
  compute_ik->setGroup("arm");
  compute_ik->setIKFrame("arm_link_wr1");
  compute_ik->setMaxIKSolutions(5000);
  compute_ik->properties().set("link_padding", 0.0);
  compute_ik->properties().set("trajectory_sampling_rate", 100);
  compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
			                                        { "target_pose" });

  // If you want to keep orientation, define orientation_tol and orientation_links properly
  bool keep_orientation = false;
  double orientation_tol = 0.01; // example tolerance
  std::vector<std::string> orientation_links;
  if (keep_orientation && !orientation_links.empty())
  {
    compute_ik->properties().set("keep_orientation", true);
    compute_ik->properties().set("keep_orientation_links", orientation_links);
    compute_ik->properties().set("orientation_tolerance", orientation_tol);
  }

  task->add(std::move(compute_ik));

  // ---------------- Move To ----------------
  planner->setMaxVelocityScalingFactor(1.0);
  planner->setMaxAccelerationScalingFactor(1.0);

  // auto move_to = std::make_unique<mtc::stages::MoveTo>("move to pose", planner);
  // move_to->setGroup("arm");
  // move_to->setIKFrame("arm_link_wr1");
  // task->add(std::move(move_to));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
