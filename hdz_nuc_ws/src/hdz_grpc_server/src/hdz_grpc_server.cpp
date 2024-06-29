#include "hdz_grpc_server.hpp"
#include "fmt_logger.hpp"

// std::unique_ptr<grpc::Server> RunGrpcServer(uint16_t port)
// {
//     std::string server_address = fmt::format("0.0.0.0:{}", port);

//     GreeterServiceImpl service;

//     grpc::EnableDefaultHealthCheckService(true);
//     // grpc::reflection::InitProtoReflectionServerBuilderPlugin();
//     ServerBuilder builder;
//     // Listen on the given address without any authentication mechanism.
//     builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
//     // Register "service" as the instance through which we'll communicate with
//     // clients. In this case it corresponds to an *synchronous* service.
//     builder.RegisterService(&service);
//     // Finally assemble the server.
//     std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
//     kLogger.Info("Server listening on {}", server_address);

//     // Wait for the server to shutdown. Note that some other thread must be
//     // responsible for shutting down the server for this call to ever return.
//     // server->Wait();
//     return server;
// }
