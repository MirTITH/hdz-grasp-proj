cmake_minimum_required(VERSION 3.8)
project(hdz_grpc_msg VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto")
set(PROTO_IMPORT_DIRS "${CMAKE_CURRENT_LIST_DIR}/proto")
file(MAKE_DIRECTORY ${PROTO_BINARY_DIR})

add_library(${PROJECT_NAME}
    proto/hdz_grpc_msg/hdz_grpc_msg.proto
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROTO_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC gRPC::grpc++)

protobuf_generate(TARGET ${PROJECT_NAME}
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}
)
protobuf_generate(
    TARGET ${PROJECT_NAME}
    LANGUAGE grpc
    PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    PLUGIN_OPTIONS generate_mock_code=true
    GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}
)

install(
    DIRECTORY ${PROTO_BINARY_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.pb.h"
)

install(DIRECTORY ${PROTO_BINARY_DIR}/
    DESTINATION src
    FILES_MATCHING PATTERN "*.pb.cc"
)

# # Create a package configuration file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# # Export the targets to a script
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)


configure_file(cmake/${PROJECT_NAME}Config.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    COPYONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# ament_package()
