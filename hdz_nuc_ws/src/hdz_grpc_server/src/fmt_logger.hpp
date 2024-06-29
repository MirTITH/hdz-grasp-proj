#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fmt/core.h>
#include <string>

class FmtLogger
{
public:
    FmtLogger(std::string name)
        : logger_(rclcpp::get_logger(name)){};

    FmtLogger(rclcpp::Node::SharedPtr node)
        : logger_(node->get_logger()){};

    template <typename... Args>
    void Info(const std::string &format_str, Args &&...args)
    {
        auto formatted_str = fmt::format(format_str, std::forward<Args>(args)...);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("hdz_grpc_server"), formatted_str);
    }

private:
    rclcpp::Logger logger_;
};

inline FmtLogger kLogger("hdz_grpc_server");
