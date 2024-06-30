#pragma once

#include <memory>
#include <fmt/core.h>
#include "fmt_logger.hpp"
#include "hdz_grpc_greeter.hpp"
#include "hdz_grpc_move_grasp.hpp"

class HdzGrpcServer
{
public:
    HdzGrpcServer(const std::string &server_address, std::shared_ptr<MoveItNode> moveit_node)
        : server_address_(server_address), move_grasp_service_(std::move(moveit_node))
    {
        grpc::EnableDefaultHealthCheckService(true);
    }

    void Start()
    {
        // grpc::reflection::InitProtoReflectionServerBuilderPlugin();
        grpc::ServerBuilder builder;
        // Listen on the given address without any authentication mechanism.
        builder.AddListeningPort(server_address_, grpc::InsecureServerCredentials());
        // Register "service" as the instance through which we'll communicate with
        // clients. In this case it corresponds to an *synchronous* service.
        builder.RegisterService(&greeter_service_);
        builder.RegisterService(&move_grasp_service_);
        // Finally assemble the server.
        server_ = builder.BuildAndStart();
        kLogger.Info("Server listening on {}", server_address_);
    }

    void Shutdown()
    {
        server_->Shutdown();
    }

    void Wait()
    {
        server_->Wait();
    }

private:
    std::string server_address_;
    GreeterServiceImpl greeter_service_;
    MoveGraspServiceImpl move_grasp_service_;
    std::unique_ptr<grpc::Server> server_;
};
