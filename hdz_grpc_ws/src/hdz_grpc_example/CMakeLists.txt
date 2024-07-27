cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_EXTENSIONS true)
set(CMAKE_C_STANDARD_REQUIRED True)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_EXTENSIONS true)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(hdz_grpc_example) # 项目名称

find_package(hdz_grpc_msg REQUIRED)

add_executable(${PROJECT_NAME}
    src/grasp_model_example.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    # 对 MSVC 开启 utf-8 支持
    "$<$<C_COMPILER_ID:MSVC>:/source-charset:utf-8>"
    "$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    hdz_grpc_msg::hdz_grpc_msg
)

target_link_options(${PROJECT_NAME} PRIVATE
    # -static
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)