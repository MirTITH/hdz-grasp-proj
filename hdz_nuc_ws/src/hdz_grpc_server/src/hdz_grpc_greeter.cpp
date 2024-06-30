#include "hdz_grpc_greeter.hpp"
#include <fmt/format.h>

using grpc::Status;

grpc::Status GreeterServiceImpl::SayHello(grpc::ServerContext *context, const ::hdz_grpc_msg::StrMsg *request, hdz_grpc_msg::StrMsg *reply)
{
    (void)context;
    std::string result = fmt::format("Hello, {}! I am hdz_grpc_server.", request->str());
    reply->set_str(result);
    return Status::OK;
}
