cmake_minimum_required(VERSION 3.8)
project(demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
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

# Executables
set(EXECUTABLES
  cartesian_movement_to_waypoints
  move_object_over_obstacle
  move_to_joint_space_goal
  move_to_pose_around_obstacle
  move_to_pose
  path_constrained_movement
)

foreach(target ${EXECUTABLES})
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} ${DEPENDENCIES})
endforeach()

# Install
install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# # Python package
# ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS demo/test_moveit_plans.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()