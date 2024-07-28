#pragma once

#include <hdz_grpc_msg/hdz_grpc_msg.grpc.pb.h>
#include <fmt/format.h>
#include <string>
#include <geometry_msgs/msg/pose.hpp>

namespace hdz_grpc_msg
{
inline std::string format_as(Pose f)
{
    return fmt::format("x: {}, y: {}, z: {}, qx: {}, qy: {}, qz: {}, qw: {}", f.position().x(), f.position().y(), f.position().z(), f.orientation().x(), f.orientation().y(), f.orientation().z(), f.orientation().w());
}
};

namespace geometry_msgs
{
namespace msg
{
inline std::string format_as(Pose f)
{
    return fmt::format("x: {}, y: {}, z: {}, qx: {}, qy: {}, qz: {}, qw: {}", f.position.x, f.position.y, f.position.z, f.orientation.x, f.orientation.y, f.orientation.z, f.orientation.w);
}
};
};
