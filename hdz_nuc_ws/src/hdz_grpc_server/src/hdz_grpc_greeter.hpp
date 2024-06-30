#pragma once

#include <grpcpp/grpcpp.h>
#include <hdz_grpc_msg/hdz_grpc_msg.grpc.pb.h>

class GreeterServiceImpl final : public hdz_grpc_msg::Greeter::Service
{
    grpc::Status SayHello(grpc::ServerContext *context, const ::hdz_grpc_msg::StrMsg *request, hdz_grpc_msg::StrMsg *reply) override;
};
