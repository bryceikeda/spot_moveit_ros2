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