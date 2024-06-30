#include <rclcpp/rclcpp.hpp>
#include "moveit_node.hpp"
#include "hdz_grpc_server.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>
#include "fmt_logger.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // MoveIt Node
    auto const moveit_node = std::make_shared<MoveItNode>("hdz_grpc_server");
    moveit_node->Init();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveit_node);
    auto executor_thread = std::thread([&executor]() { executor.spin(); });

    kLogger.Info("Planning frame: {}", moveit_node->move_group_->getPlanningFrame().c_str());
    kLogger.Info("End effector link: {}", moveit_node->move_group_->getEndEffectorLink().c_str());
    kLogger.Info("Available Planning Groups: {}", moveit_node->move_group_->getJointModelGroupNames());

    // gRPC Server
    std::string grpc_server_address = "0.0.0.0:9999"; // This is the default value
    moveit_node->get_parameter("grpc_server_address", grpc_server_address);

    kLogger.Info("Starting grpc server on {}", grpc_server_address);
    HdzGrpcServer grpc_server{grpc_server_address, moveit_node};
    grpc_server.Start();

    // Wait for executor thread to finish
    executor_thread.join();

    grpc_server.Shutdown();
    rclcpp::shutdown();
    return 0;
}
