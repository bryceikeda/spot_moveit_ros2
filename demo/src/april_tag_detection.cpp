#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "spot_behavior_msgs/action/detect_april_tags.hpp"
#include "spot_behavior_msgs/msg/april_tag_detection.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"


class AprilTagDetection : public rclcpp::Node
{
public:
    using DetectAprilTags = spot_behavior_msgs::action::DetectAprilTags;
    using GoalHandleDetect = rclcpp_action::ClientGoalHandle<DetectAprilTags>;

    AprilTagDetection() 
        : Node("april_tag_planning_scene_client")
    {
        client_ = rclcpp_action::create_client<DetectAprilTags>(
            this,   // ✔ pass raw this pointer
            "/perception/detect_april_tags"
        );

        RCLCPP_INFO(get_logger(), "Waiting for DetectAprilTags action server...");
        client_->wait_for_action_server();

    }
    void send_goal()
    {
        auto goal_msg = DetectAprilTags::Goal();

        RCLCPP_INFO(get_logger(), "Sending DetectAprilTags goal...");

        rclcpp_action::Client<DetectAprilTags>::SendGoalOptions options;

        options.goal_response_callback =
            std::bind(&AprilTagDetection::goal_response_callback, this, std::placeholders::_1);

        options.feedback_callback =
            std::bind(&AprilTagDetection::feedback_callback, this,
                    std::placeholders::_1, std::placeholders::_2);

        options.result_callback =
            std::bind(&AprilTagDetection::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, options);
    }

private:
    rclcpp_action::Client<DetectAprilTags>::SharedPtr client_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    void goal_response_callback(const GoalHandleDetect::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal rejected by server.");
        } else {
            RCLCPP_INFO(get_logger(), "Goal accepted!");
        }
    }

    void feedback_callback(
        GoalHandleDetect::SharedPtr,
        const std::shared_ptr<const DetectAprilTags::Feedback> feedback)
    {
        (void)feedback; // no feedback in this action
    }

    void result_callback(const GoalHandleDetect::WrappedResult & result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(get_logger(), "DetectAprilTags action failed or was aborted.");
            return;
        }

        auto detections = result.result->detections;
        RCLCPP_INFO(get_logger(), "Received %zu AprilTag detections.", detections.size());

        add_tags_to_planning_scene(detections);
    }

    void add_tags_to_planning_scene(const std::vector<spot_behavior_msgs::msg::AprilTagDetection> & detections)
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        for (const auto & det : detections) {
            moveit_msgs::msg::CollisionObject obj;
            obj.header = det.header;
            obj.id = "apriltag_" + std::to_string(det.id);

            // simple box primitive representing the tag
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {0.20, 0.20, 0.01};  // size of the AprilTag

            obj.primitives.push_back(primitive);
            obj.primitive_poses.push_back(det.pose); // already a geometry_msgs/Pose
            obj.operation = obj.ADD;

            collision_objects.push_back(obj);

            RCLCPP_INFO(
                get_logger(),
                "Added AprilTag object id=%d at (%.3f, %.3f, %.3f)",
                det.id,
                det.pose.position.x,
                det.pose.position.y,
                det.pose.position.z);
        }

        planning_scene_.addCollisionObjects(collision_objects);

        RCLCPP_INFO(get_logger(), "All AprilTags added to the planning scene.");
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDetection>();

    // Create a timer inside the node to periodically send goals
    auto timer = node->create_wall_timer(
        std::chrono::seconds(1),  // send goal every 1 second
        [node]() {
            node->send_goal();
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
