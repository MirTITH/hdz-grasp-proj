cmake_minimum_required(VERSION 3.8)
project(hdz_grpc_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "../../non_ros_pkgs/install")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hdz_grpc_msg REQUIRED)
find_package(fmt REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/hdz_grpc_greeter.cpp
  src/hdz_grpc_arm.cpp
  src/gripper_node.cpp
  src/moveit_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  # $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  control_msgs
)

target_link_libraries(${PROJECT_NAME}
  hdz_grpc_msg::hdz_grpc_msg
  fmt::fmt
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
