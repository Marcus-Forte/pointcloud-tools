

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "filter_services.h"
#include "metric_services.h"

using grpc::Channel;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

int main() {
  std::string server_address = "0.0.0.0:50052";
  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();

  MetricServicesImpl service;
  FilterServicesImpl filter_service;

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  builder.RegisterService(&filter_service);

  std::unique_ptr<Server> server(builder.BuildAndStart());

  std::cout << "Listening to: " << server_address << std::endl;

  server->Wait();

  return 0;
}