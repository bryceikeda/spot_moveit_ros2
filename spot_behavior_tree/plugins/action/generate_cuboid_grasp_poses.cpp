#include "spot_behavior_tree/plugins/action/generate_cuboid_grasp_poses.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h> 
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

BT_REGISTER_NODES(factory)
{
  GenerateCuboidGraspPoses::RegisterNodes(factory);
}
static geometry_msgs::msg::Pose applyEeToTcpOffset(
    const geometry_msgs::msg::Pose& tcp_pose,
    const geometry_msgs::msg::Pose& ee_to_tcp)
{
    tf2::Transform T_obj_tcp, T_ee_tcp;
    tf2::fromMsg(tcp_pose, T_obj_tcp);
    tf2::fromMsg(ee_to_tcp, T_ee_tcp);

    tf2::Transform T_tcp_ee = T_ee_tcp.inverse();
    tf2::Transform T_obj_ee = T_obj_tcp * T_tcp_ee;

    geometry_msgs::msg::Pose out;
    out.position.x = T_obj_ee.getOrigin().x();
    out.position.y = T_obj_ee.getOrigin().y();
    out.position.z = T_obj_ee.getOrigin().z();
    out.orientation = tf2::toMsg(T_obj_ee.getRotation());
    return out;
}

static geometry_msgs::msg::Pose makePose(
    const geometry_msgs::msg::Pose& obj_pose,
    double dx, double dy, double dz,
    double rx, double ry, double rz)
{
    tf2::Transform T_obj, T_offset;
    tf2::fromMsg(obj_pose, T_obj);

    tf2::Quaternion q;
    q.setRPY(rx, ry, rz);

    T_offset.setOrigin(tf2::Vector3(dx, dy, dz));
    T_offset.setRotation(q);

    tf2::Transform T = T_obj * T_offset;
    geometry_msgs::msg::Pose p;
    p.position.x = T.getOrigin().x();
    p.position.y = T.getOrigin().y();
    p.position.z = T.getOrigin().z();
    p.orientation = tf2::toMsg(T.getRotation());
    return p;
}

namespace GenerateCuboidGraspPoses{
    GenerateCuboidGraspPoses::GenerateCuboidGraspPoses(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus GenerateCuboidGraspPoses::tick()
    {
        bool generate_x_axis_grasps, generate_y_axis_grasps, generate_z_axis_grasps;
        getInput<bool>("generate_x_axis_grasps", generate_x_axis_grasps);
        getInput<bool>("generate_y_axis_grasps", generate_y_axis_grasps);
        getInput<bool>("generate_z_axis_grasps", generate_z_axis_grasps);

        geometry_msgs::msg::PoseStamped ee_to_tcp;
        if (!getInput("ee_to_tcp", ee_to_tcp))
            throw BT::RuntimeError("GenerateCuboidGraspPoses -> Missing [ee_to_tcp]");

        moveit_msgs::msg::CollisionObject target_object; 
        if (!getInput<moveit_msgs::msg::CollisionObject>("target_object", target_object))
            throw BT::RuntimeError("GenerateCollisionObjectGraspPoses -> Missing required input [target_object]");
     
        if (target_object.primitives.empty())
            throw BT::RuntimeError("CollisionObject has no primitives");

        const auto& box = target_object.primitives[0];
        const auto& pose = target_object.pose;

        double x = box.dimensions[0] * 0.5;
        double y = box.dimensions[1] * 0.5;
        double z = box.dimensions[2] * 0.5;

        std::vector<geometry_msgs::msg::PoseStamped> grasps;

        auto add = [&](double dx, double dy, double dz,
                    double rx, double ry, double rz)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = target_object.header.frame_id;

            // Object -> TCP
            geometry_msgs::msg::Pose tcp_pose = makePose(pose, dx, dy, dz, rx, ry, rz);

            // Convert to Object -> EE using ee_to_tcp
            ps.pose = applyEeToTcpOffset(tcp_pose, ee_to_tcp.pose);
            
            grasps.push_back(ps);
        };

        // +X / -X
        if (generate_x_axis_grasps)
        {
            add(+x, 0, 0, 0, 0, -M_PI);   // approach -X
            add(-x, 0, 0, 0, 0, 0);   // approach +X
        }

        // +Y / -Y
        if (generate_y_axis_grasps)
        {
            add(0, +y, 0, 0, 0, -M_PI/2);
            add(0, -y, 0, 0, 0, M_PI/2);
        }

        // +Z / -Z
        if (generate_z_axis_grasps)
        {
            add(0, 0, +z, 0, M_PI/2, 0);
            add(0, 0, -z, 0, -M_PI/2, 0);
        }

        setOutput("grasp_poses", grasps);
        return BT::NodeStatus::SUCCESS;


    }

    BT::PortsList GenerateCuboidGraspPoses::providedPorts()
    {
        return {
            BT::InputPort<moveit_msgs::msg::CollisionObject>("target_object"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("ee_to_tcp"),
            BT::InputPort<bool>("generate_z_axis_grasps"),
            BT::InputPort<bool>("generate_y_axis_grasps"),
            BT::InputPort<bool>("generate_x_axis_grasps"),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("grasp_poses")
        };
    }
}

