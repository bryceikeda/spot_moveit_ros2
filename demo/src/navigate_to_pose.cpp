#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class NavigationActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigationActionClient()
  : Node("navigate_to_pose_back")
  {
    // These MUST be declared here
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("yaw", 0.0);
    this->declare_parameter<std::string>("frame_id", "vision");
    client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "/navigation/navigate_to_pose"
    );
  }
  void start_navigation()
  {
    float x = this->get_parameter("x").as_double();
    float y = this->get_parameter("y").as_double();
    float yaw = this->get_parameter("yaw").as_double();
    std::string frame_id = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO(this->get_logger(),
      "NavigateToPose: x=%.2f y=%.2f yaw=%.2f",
      x, y, yaw);

    send_goal(x, y, yaw, frame_id);
  }
  void send_goal(float x, float y, float yaw_deg = 0.0, std::string frame_id="vision")
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server available. Sending goal...");

    NavigateToPose::Goal goal_msg;

    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = frame_id; 
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;

    // convert yaw to quaternion
    float yaw_rad = yaw_deg * M_PI / 180.0f;
    goal_msg.pose.pose.orientation.z = std::sin(yaw_rad / 2.0f);
    goal_msg.pose.pose.orientation.w = std::cos(yaw_rad / 2.0f);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Distance remaining: %.2f",
          feedback->distance_remaining
        );
      };

    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation canceled.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationActionClient>();

  node->start_navigation();
  rclcpp::spin(node);

  return 0;
}
