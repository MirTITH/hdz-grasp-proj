#pragma once

#include <rclcpp/rclcpp.hpp>

class GrpcConfigNode : public rclcpp::Node
{
public:
    GrpcConfigNode(const std::string &node_name)
        : Node(node_name)
    {
        this->declare_parameter("grpc_server_address", "0.0.0.0:9999");
    }
};