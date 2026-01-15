cmake_minimum_required(VERSION 3.16)
project(spot_behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
    behaviortree_ros2
    std_msgs
    std_srvs
    btcpp_ros2_interfaces
    rclcpp
    rclcpp_action
    rclcpp_components
    moveit_ros_planning_interface
    moveit_core
    control_msgs
    moveit_msgs
    moveit_visual_tools
    moveit_task_constructor_core
    moveit_task_constructor_msgs
)

foreach(package ${DEPENDENCIES})
  find_package(${package} REQUIRED)
endforeach()

######################################################
# Simple example showing how to use and customize the BtExecutionServer
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor ${DEPENDENCIES})

######################################################
# Build a client that call the sleep action (Plugin version)
add_library(execute_trajectory_plugin SHARED plugins/action/execute_trajectory.cpp)
list(APPEND plugin_libs execute_trajectory_plugin)

# add_library(plan_mtc_cartesian_path_plugin SHARED plugins/action/plan_mtc_cartesian_path.cpp)
# list(APPEND plugin_libs plan_mtc_cartesian_path_plugin)

add_library(transform_pose_plugin SHARED plugins/action/transform_pose.cpp)
list(APPEND plugin_libs transform_pose_plugin)

add_library(generate_cuboid_grasp_poses_plugin SHARED plugins/action/generate_cuboid_grasp_poses.cpp)
list(APPEND plugin_libs generate_cuboid_grasp_poses_plugin)

add_library(get_collision_object_from_planning_scene_plugin SHARED plugins/action/get_collision_object_from_planning_scene.cpp)
list(APPEND plugin_libs get_collision_object_from_planning_scene_plugin)

add_library(setup_mtc_ignore_graspable_collisions_plugin SHARED plugins/action/setup_mtc_ignore_graspable_collisions.cpp)
list(APPEND plugin_libs setup_mtc_ignore_graspable_collisions_plugin)

add_library(reset_string_vector_plugin SHARED plugins/action/reset_string_vector.cpp)
list(APPEND plugin_libs reset_string_vector_plugin)

add_library(add_to_string_vector_plugin SHARED plugins/action/add_to_string_vector.cpp)
list(APPEND plugin_libs add_to_string_vector_plugin)

add_library(is_object_attached_to_plugin SHARED plugins/action/is_object_attached_to.cpp)
list(APPEND plugin_libs is_object_attached_to_plugin)

add_library(setup_mtc_attach_object_plugin SHARED plugins/action/setup_mtc_attach_object.cpp)
list(APPEND plugin_libs setup_mtc_attach_object_plugin)

add_library(setup_mtc_detach_object_plugin SHARED plugins/action/setup_mtc_detach_object.cpp)
list(APPEND plugin_libs setup_mtc_detach_object_plugin)

add_library(setup_mtc_ignore_collisions_between_objects_plugin SHARED plugins/action/setup_mtc_ignore_collisions_between_objects.cpp)
list(APPEND plugin_libs setup_mtc_ignore_collisions_between_objects_plugin)

add_library(reset_vector_plugin SHARED plugins/action/reset_vector.cpp)
list(APPEND plugin_libs reset_vector_plugin)

add_library(add_to_vector_plugin SHARED plugins/action/add_to_vector.cpp)
list(APPEND plugin_libs add_to_vector_plugin)

add_library(reset_pose_stamped_vector_plugin SHARED plugins/action/reset_pose_stamped_vector.cpp)
list(APPEND plugin_libs reset_pose_stamped_vector_plugin)

add_library(setup_mtc_batch_pose_ik_plugin SHARED plugins/action/setup_mtc_batch_pose_ik.cpp)
list(APPEND plugin_libs setup_mtc_batch_pose_ik_plugin)

add_library(setup_mtc_update_object_collision_rule_plugin SHARED plugins/action/setup_mtc_update_object_collision_rule.cpp)
list(APPEND plugin_libs setup_mtc_update_object_collision_rule_plugin)

add_library(setup_mtc_move_along_frame_axis_plugin SHARED plugins/action/setup_mtc_move_along_frame_axis.cpp)
list(APPEND plugin_libs setup_mtc_move_along_frame_axis_plugin)

add_library(setup_mtc_connect_with_rrt_plugin SHARED plugins/action/setup_mtc_connect_with_rrt.cpp)
list(APPEND plugin_libs setup_mtc_connect_with_rrt_plugin)

add_library(execute_mtc_task_plugin SHARED plugins/action/execute_mtc_task.cpp)
list(APPEND plugin_libs execute_mtc_task_plugin)

add_library(plan_mtc_task_plugin SHARED plugins/action/plan_mtc_task.cpp)
list(APPEND plugin_libs plan_mtc_task_plugin)

add_library(setup_mtc_plan_to_pose_plugin SHARED plugins/action/setup_mtc_plan_to_pose.cpp)
list(APPEND plugin_libs setup_mtc_plan_to_pose_plugin)

add_library(execute_moveit_trajectory_plugin SHARED plugins/action/execute_moveit_trajectory.cpp)
list(APPEND plugin_libs execute_moveit_trajectory_plugin)

add_library(plan_moveit_target_plugin SHARED plugins/action/plan_moveit_target.cpp)
list(APPEND plugin_libs plan_moveit_target_plugin)

add_library(set_moveit_pose_stamped_target_plugin SHARED plugins/action/set_moveit_pose_stamped_target.cpp)
list(APPEND plugin_libs set_moveit_pose_stamped_target_plugin)

add_library(set_moveit_joint_value_target_plugin SHARED plugins/action/set_moveit_joint_value_target.cpp)
list(APPEND plugin_libs set_moveit_joint_value_target_plugin)

add_library(create_stamped_pose_plugin SHARED plugins/action/create_stamped_pose.cpp)
list(APPEND plugin_libs create_stamped_pose_plugin)

add_library(add_pose_stamped_to_vector_plugin SHARED plugins/action/add_pose_stamped_to_vector.cpp)
list(APPEND plugin_libs add_pose_stamped_to_vector_plugin)

add_library(setup_move_group_plugin SHARED plugins/action/setup_move_group.cpp)
list(APPEND plugin_libs setup_move_group_plugin)

add_library(create_joint_state_plugin SHARED plugins/action/create_joint_state.cpp)
list(APPEND plugin_libs create_joint_state_plugin)

add_library(setup_visual_tools_plugin SHARED plugins/action/setup_visual_tools.cpp)
list(APPEND plugin_libs setup_visual_tools_plugin)

add_library(visualize_pose_plugin SHARED plugins/action/visualize_pose.cpp)
list(APPEND plugin_libs visualize_pose_plugin)

add_library(visualize_waypoints_plugin SHARED plugins/action/visualize_waypoints.cpp)
list(APPEND plugin_libs visualize_waypoints_plugin)

add_library(clear_markers_plugin SHARED plugins/action/clear_markers.cpp)
list(APPEND plugin_libs clear_markers_plugin)

add_library(plan_moveit_cartesian_path_plugin SHARED plugins/action/plan_moveit_cartesian_path.cpp)
list(APPEND plugin_libs plan_moveit_cartesian_path_plugin)

add_library(create_solid_primitive_box_plugin SHARED plugins/action/create_solid_primitive_box.cpp)
list(APPEND plugin_libs create_solid_primitive_box_plugin)

add_library(create_collision_object_from_solid_primitive_plugin SHARED plugins/action/create_collision_object_from_solid_primitive.cpp)
list(APPEND plugin_libs create_collision_object_from_solid_primitive_plugin)

add_library(add_collision_object_to_planning_scene_plugin SHARED plugins/action/add_collision_object_to_planning_scene.cpp)
list(APPEND plugin_libs add_collision_object_to_planning_scene_plugin)

add_library(is_collision_object_in_planning_scene_plugin SHARED plugins/action/is_collision_object_in_planning_scene.cpp)
list(APPEND plugin_libs is_collision_object_in_planning_scene_plugin)

add_library(get_current_planning_scene_plugin SHARED plugins/action/get_current_planning_scene.cpp)
list(APPEND plugin_libs get_current_planning_scene_plugin)

add_library(visualize_trajectory_plugin SHARED plugins/action/visualize_trajectory.cpp)
list(APPEND plugin_libs visualize_trajectory_plugin)

add_library(add_ros_node_to_blackboard_plugin SHARED plugins/action/add_ros_node_to_blackboard.cpp)
list(APPEND plugin_libs add_ros_node_to_blackboard_plugin)

add_library(set_moveit_named_target_plugin SHARED plugins/action/set_moveit_named_target.cpp)
list(APPEND plugin_libs set_moveit_named_target_plugin)

add_library(attach_object_to_link_plugin SHARED plugins/action/attach_object_to_link.cpp)
list(APPEND plugin_libs attach_object_to_link_plugin)

add_library(detach_object_from_link_plugin SHARED plugins/action/detach_object_from_link.cpp)
list(APPEND plugin_libs detach_object_from_link_plugin)

add_library(initialize_motion_constraints_plugin SHARED plugins/action/initialize_motion_constraints.cpp)
list(APPEND plugin_libs initialize_motion_constraints_plugin)

add_library(append_orientation_constraint_plugin SHARED plugins/action/append_orientation_constraint.cpp)
list(APPEND plugin_libs append_orientation_constraint_plugin)

add_library(set_moveit_planning_time_plugin SHARED plugins/action/set_moveit_planning_time.cpp)
list(APPEND plugin_libs set_moveit_planning_time_plugin)

add_library(initialize_mtc_task_plugin SHARED plugins/action/initialize_mtc_task.cpp)
list(APPEND plugin_libs initialize_mtc_task_plugin)

add_library(setup_mtc_current_state_plugin SHARED plugins/action/setup_mtc_current_state.cpp)
list(APPEND plugin_libs setup_mtc_current_state_plugin)

add_library(move_gripper_action_plugin SHARED plugins/action/move_gripper_action.cpp)
list(APPEND plugin_libs move_gripper_action_plugin)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${bt_plugin} ${DEPENDENCIES})
  set_target_properties(${bt_plugin} PROPERTIES PREFIX "")
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

######################################################
# INSTALL

install(TARGETS
  bt_executor  
  DESTINATION lib/${PROJECT_NAME}
  )

######################################################
# INSTALL plugins for other packages to load

install(TARGETS
  ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

######################################################
# INSTALL Behavior.xml's, ROS config and launch files

install(DIRECTORY
    behavior_trees
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()