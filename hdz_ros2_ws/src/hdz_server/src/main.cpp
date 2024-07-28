#include <rclcpp/rclcpp.hpp>
#include "moveit_node.hpp"
#include "gripper_node.hpp"
#include "hdz_server.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    // MoveIt Node
    auto const moveit_node = std::make_shared<MoveItNode>("hdz_server_moveit_node");
    moveit_node->Init();
    executor.add_node(moveit_node);

    // Gripper Node
    auto const gripper_node = std::make_shared<GripperNode>("hdz_server_gripper_node");
    executor.add_node(gripper_node);

    // HDZ Server
    auto const hdz_server = std::make_shared<HdzServer>("hdz_server", moveit_node, gripper_node);
    executor.add_node(hdz_server);

    // // gRPC Config Node
    // auto const grpc_config_node = std::make_shared<GrpcConfigNode>("hdz_server_grpc_config_node");
    // executor.add_node(grpc_config_node);

    // // gRPC Server
    // auto grpc_server_address = grpc_config_node->get_parameter("grpc_server_address").as_string();
    // kLogger.Info("Starting grpc server on {}", grpc_server_address);
    // HdzGrpcServer grpc_server{grpc_server_address, moveit_node, gripper_node};
    // grpc_server.Start();

    // Wait for executor thread to finish
    // auto executor_thread = std::thread([&executor]() { executor.spin(); });
    // executor_thread.join();
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
