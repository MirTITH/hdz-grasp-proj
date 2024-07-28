#pragma once

#include <grpcpp/grpcpp.h>
#include <hdz_grpc_msg/hdz_grpc_msg.grpc.pb.h>
#include "moveit_node.hpp"
#include <memory>
#include "gripper_node.hpp"

class ArmServiceImpl final : public hdz_grpc_msg::Arm::Service
{
public:
    ArmServiceImpl(std::shared_ptr<MoveItNode> moveit_node,
                   std::shared_ptr<GripperNode> gripper_node)
        : moveit_node_(std::move(moveit_node)),
          gripper_node_(std::move(gripper_node)){};

    virtual ::grpc::Status MoveTo(::grpc::ServerContext *context, const ::hdz_grpc_msg::PoseStamped *request, ::hdz_grpc_msg::BoolWithMsg *response) override;
    virtual ::grpc::Status MoveToJointPosition(::grpc::ServerContext *context, const ::hdz_grpc_msg::Array *request, ::hdz_grpc_msg::BoolWithMsg *response) override;
    virtual ::grpc::Status MoveToNamed(::grpc::ServerContext *context, const ::hdz_grpc_msg::StrMsg *request, ::hdz_grpc_msg::BoolWithMsg *response) override;
    virtual ::grpc::Status SetGripper(::grpc::ServerContext *context, const ::hdz_grpc_msg::SetGripperRequest *request, ::hdz_grpc_msg::BoolWithMsg *response) override;
    virtual ::grpc::Status GetPose(::grpc::ServerContext *context, const ::hdz_grpc_msg::StrMsg *request, ::hdz_grpc_msg::PoseStamped *response) override {};
    virtual ::grpc::Status GetJointPosition(::grpc::ServerContext *context, const ::google::protobuf::Empty *request, ::hdz_grpc_msg::Array *response) override {};

private:
    std::shared_ptr<MoveItNode> moveit_node_;
    std::shared_ptr<GripperNode> gripper_node_;
    FmtLogger logger_{"ArmServiceImpl"};
};