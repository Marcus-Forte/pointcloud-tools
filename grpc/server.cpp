#include "helloworld.grpc.pb.h"

#include <memory>
#include <grpcpp/grpcpp.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

using grpc::Channel;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

class GreeterServiceImpl final : public Greeter::Service
{
    Status SayHello(ServerContext *context, const HelloRequest *request, HelloReply *reply) override
    {
        std::string prefix("Hello from server!!!");
        reply->set_message(prefix + request->name());

        std::cout << "Received: " << request->name() << std::endl;
        std::cout << "Sent: " << reply->message() << std::endl;
        return Status::OK;
    }
};

int main()
{

    std::string server_address = "0.0.0.0:50051";
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();

    GreeterServiceImpl service;

    ServerBuilder builder;

    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    std::unique_ptr<Server> server(builder.BuildAndStart());

    std::cout << "Listening...\n";
    server->Wait();

    return 0;
}