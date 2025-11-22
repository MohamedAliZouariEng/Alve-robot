cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)

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

# Generate the executable
add_executable(pick_place_good src/pick_place_good.cpp src/get_pose_client.cpp src/robot_controller.cpp)
target_include_directories(pick_place_good PUBLIC include)
ament_target_dependencies(pick_place_good
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs
  grasping_msgs
)

# Install the executable
install(TARGETS pick_place_good DESTINATION lib/${PROJECT_NAME}
)



add_executable(pick_place_right src/pick_place_right.cpp src/get_pose_client.cpp src/robot_controller.cpp)
target_include_directories(pick_place_right PUBLIC include)
ament_target_dependencies(pick_place_right
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs
  grasping_msgs
)

# Install the executable
install(TARGETS pick_place_right DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY
launch
 DESTINATION share/${PROJECT_NAME}
)

ament_package()