#pragma once

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <string>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <array>

template <>
struct fmt::formatter<std_msgs::msg::Header> : formatter<std::string> {
    format_context::iterator format(std_msgs::msg::Header h, format_context &ctx) const
    {
        auto str = fmt::format("frame_id: {}, stamp: {}.{}", h.frame_id, h.stamp.sec, h.stamp.nanosec);
        return formatter<std::string>::format(str, ctx);
    }
};

template <>
struct fmt::formatter<geometry_msgs::msg::Pose> : formatter<std::string> {
    format_context::iterator format(geometry_msgs::msg::Pose pose, format_context &ctx) const
    {
        std::array<double, 3> positions    = {pose.position.x, pose.position.y, pose.position.z};
        std::array<double, 4> orientations = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};

        auto str = fmt::format("position: {}, orientation: {}", positions, orientations);
        return formatter<std::string>::format(str, ctx);
    }
};

template <>
struct fmt::formatter<geometry_msgs::msg::PoseStamped> : formatter<std::string> {
    format_context::iterator format(geometry_msgs::msg::PoseStamped pose_stamped, format_context &ctx) const
    {
        auto str = fmt::format("header: {}, pose: {}", pose_stamped.header, pose_stamped.pose);
        return formatter<std::string>::format(str, ctx);
    }
};
