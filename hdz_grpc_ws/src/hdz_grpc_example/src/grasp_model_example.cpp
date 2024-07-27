#include <grpcpp/grpcpp.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/health_check_service_interface.h>
#include <hdz_grpc_msg/grasp_model.grpc.pb.h>
#include <iostream>
#include <thread>

using grasp_model::Greeter;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using hdz_grpc_common::StrMsg;

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;

class GreeterServiceImpl final : public Greeter::Service
{
    Status SayHello(ServerContext *, const StrMsg *request, StrMsg *reply) override
    {
        std::string prefix("Hello ");
        reply->set_str(prefix + request->str());
        return Status::OK;
    }
};

class GreeterClient
{
public:
    GreeterClient(std::shared_ptr<Channel> channel)
        : stub_(Greeter::NewStub(channel))
    {
    }

    // Assembles the client's payload, sends it and presents the response back
    // from the server.
    std::string SayHello(const std::string &user)
    {
        // Data we are sending to the server.
        StrMsg request;
        request.set_str(user);

        // Container for the data we expect from the server.
        StrMsg reply;

        // Context for the client. It could be used to convey extra information to
        // the server and/or tweak certain RPC behaviors.
        ClientContext context;

        // The actual RPC.
        Status status = stub_->SayHello(&context, request, &reply);

        // Act upon its status.
        if (status.ok()) {
            return reply.str();
        } else {
            std::cout << status.error_code() << ": " << status.error_message()
                      << std::endl;
            return "RPC failed";
        }
    }

private:
    std::unique_ptr<Greeter::Stub> stub_;
};

std::unique_ptr<Server> kServer;

void RunServer(uint16_t port)
{
    std::string server_address = absl::StrFormat("0.0.0.0:%d", port);
    GreeterServiceImpl service;

    grpc::EnableDefaultHealthCheckService(true);
    // grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Finally assemble the server.
    kServer = builder.BuildAndStart();
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    kServer->Wait();
}

int main(int, char **)
{
    std::cout << "Starting server..." << std::endl;
    std::thread server_thread(RunServer, 50051);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Starting client..." << std::endl;
    GreeterClient greeter(grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials()));
    std::string user("world");
    std::string reply = greeter.SayHello(user);
    std::cout << "Greeter received: " << reply << std::endl;

    std::cout << "Shutting down server..." << std::endl;
    kServer->Shutdown();
    server_thread.join();

    return 0;
}
