cmake_minimum_required(VERSION 3.8)
project(hdz_grpc_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "../../non_ros_pkgs/install/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hdz_grpc_msg REQUIRED)
find_package(fmt REQUIRED)

add_executable(${PROJECT_NAME}
  src/hdz_grpc_server.cpp
)

target_link_libraries(${PROJECT_NAME}
  hdz_grpc_msg::hdz_grpc_msg
  fmt::fmt
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
