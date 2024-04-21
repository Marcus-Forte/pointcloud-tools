

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "filter_services.h"
#include "metric_services.h"
#ifdef WITH_COLMAP
#include "reconstruct_services.h"
#endif

using grpc::Channel;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

void RunServer() 
{
  std::string server_address = "0.0.0.0:50052";
  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();

  MetricServicesImpl metric_service;
  FilterServicesImpl filter_service;

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&metric_service);
  builder.RegisterService(&filter_service);

#ifdef WITH_COLMAP
  ReconstructServiceImpl reconstruct_service;
  builder.RegisterService(&reconstruct_service);
#endif

  std::unique_ptr<Server> server(builder.BuildAndStart());

  std::cout << "Listening to: " << server_address << std::endl;

  server->Wait();
}

int main() {
  RunServer();
  
  return 0;
}