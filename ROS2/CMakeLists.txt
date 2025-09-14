cmake_minimum_required(VERSION 3.8)
project(ur10e_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(ur10e_home_and_up src/ur10e_home_and_up.cpp)
ament_target_dependencies(ur10e_home_and_up 
  rclcpp 
  moveit_ros_planning_interface 
  geometry_msgs
  moveit_core
)

add_executable(ur10e_home_and_points src/ur10e_home_and_points.cpp)
ament_target_dependencies(ur10e_home_and_points
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_core
)

add_executable(tourner src/tourner.cpp)
ament_target_dependencies(tourner
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_core
)


install(TARGETS 
  ur10e_home_and_up
  ur10e_home_and_points
  tourner
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
