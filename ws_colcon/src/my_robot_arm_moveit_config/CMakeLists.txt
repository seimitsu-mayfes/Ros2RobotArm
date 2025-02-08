cmake_minimum_required(VERSION 3.8)
project(my_robot_arm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  DEPENDENCIES geometry_msgs
)

find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(std_srvs REQUIRED)

# モーションプランナーノードの実行ファイルを追加
add_executable(motion_planner_node src/motion_planner_node.cpp)
target_include_directories(motion_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planner_node PUBLIC c_std_99 cxx_std_17)
rosidl_target_interfaces(motion_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(motion_planner_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_visualization
  moveit_visual_tools
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  rviz_visual_tools
  std_srvs
)

# 設定ファイルとlaunchファイルのインストール
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 実行ファイルのインストール
install(TARGETS motion_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
