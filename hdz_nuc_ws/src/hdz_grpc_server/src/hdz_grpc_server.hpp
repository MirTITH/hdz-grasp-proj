#pragma once

#include <memory>
#include <grpcpp/grpcpp.h>
#include <hdz_grpc_msg/helloworld.grpc.pb.h>
#include <fmt/core.h>
#include "fmt_logger.hpp"

// std::unique_ptr<grpc::Server> RunGrpcServer(uint16_t port);

class GreeterServiceImpl final : public helloworld::Greeter::Service
{
    grpc::Status SayHello(grpc::ServerContext *context, const helloworld::HelloRequest *request, helloworld::HelloReply *reply) override
    {
        (void)context;
        std::string prefix("Hello from hdz_grpc_server.");
        reply->set_message(prefix + request->name());
        return grpc::Status::OK;
    }
};

class HdzGrpcServer
{
public:
    HdzGrpcServer(const std::string &server_address = "0.0.0.0:8888")
        : server_address_(server_address)
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
        builder.RegisterService(&service_);
        // Finally assemble the server.
        server_ = builder.BuildAndStart();
        kLogger.Info("Server listening on {}", server_address_);
    }

    void Shutdown()
    {
        server_->Shutdown();
    }

private:
    GreeterServiceImpl service_;
    std::unique_ptr<grpc::Server> server_;
    std::string server_address_;
};
