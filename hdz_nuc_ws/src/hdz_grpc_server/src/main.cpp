#include <rclcpp/rclcpp.hpp>
#include "moveit_node.hpp"
#include "hdz_grpc_server.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>
#include "fmt_logger.hpp"
#include "gripper_node.hpp"
#include "grpc_config_node.hpp"

static FmtLogger kLogger("hdz_grpc_server::main");

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    // MoveIt Node
    auto const moveit_node = std::make_shared<MoveItNode>("hdz_grpc_server_moveit_node");
    moveit_node->Init();
    executor.add_node(moveit_node);

    // Gripper Node
    auto const gripper_node = std::make_shared<GripperNode>("hdz_grpc_server_gripper_node");
    executor.add_node(gripper_node);

    // gRPC Config Node
    auto const grpc_config_node = std::make_shared<GrpcConfigNode>("hdz_grpc_server_grpc_config_node");
    executor.add_node(grpc_config_node);

    // gRPC Server
    auto grpc_server_address = grpc_config_node->get_parameter("grpc_server_address").as_string();
    kLogger.Info("Starting grpc server on {}", grpc_server_address);
    HdzGrpcServer grpc_server{grpc_server_address, moveit_node, gripper_node};
    grpc_server.Start();

    // Wait for executor thread to finish
    // auto executor_thread = std::thread([&executor]() { executor.spin(); });
    // executor_thread.join();
    executor.spin();

    grpc_server.Shutdown();
    rclcpp::shutdown();
    return 0;
}
