#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fmt/core.h>
#include <string>

class FmtLogger
{
public:
    FmtLogger(std::string name = "unnamed")
        : logger_(rclcpp::get_logger(name)){};

    FmtLogger(rclcpp::Node::SharedPtr node)
        : logger_(node->get_logger()){};

    FmtLogger(rclcpp::Logger logger)
        : logger_(std::move(logger)){};

    template <typename... Args>
    void Info(const std::string &format_str, Args &&...args)
    {
        auto formatted_str = fmt::format(format_str, std::forward<Args>(args)...);
        RCLCPP_INFO_STREAM(logger_, formatted_str);
    }

    template <typename... Args>
    void Error(const std::string &format_str, Args &&...args)
    {
        auto formatted_str = fmt::format(format_str, std::forward<Args>(args)...);
        RCLCPP_ERROR_STREAM(logger_, formatted_str);
    }

    template <typename... Args>
    void Warn(const std::string &format_str, Args &&...args)
    {
        auto formatted_str = fmt::format(format_str, std::forward<Args>(args)...);
        RCLCPP_WARN_STREAM(logger_, formatted_str);
    }

private:
    rclcpp::Logger logger_;
};
