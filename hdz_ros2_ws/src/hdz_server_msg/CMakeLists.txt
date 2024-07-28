cmake_minimum_required(VERSION 3.8)
project(hdz_server_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# GLOB_RECURSE: 递归搜索
# CONFIGURE_DEPENDS: 每次执行 cmake 时都更新列表
# RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}：生成相对于 ${CMAKE_CURRENT_SOURCE_DIR} 的路径
# （rosidl_generate_interfaces 只接受相对路径）
file(GLOB_RECURSE SRVS CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  msg/*.msg
  srv/*.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRVS}
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
