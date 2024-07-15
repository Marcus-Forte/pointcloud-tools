

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "filter_services.h"
#include "metric_services.h"
#include "reconstruct_services.h"

using grpc::Server;
using grpc::ServerBuilder;

int main(int argc, char** argv) {
  const std::string server_address = "0.0.0.0:50052";
  bool use_gpu = false;

  if (argc > 2) {
    use_gpu = std::atoi(argv[1]);
  }

  std::cout << "Use GPU: " << use_gpu << std::endl;

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();

  duna::MetricServicesImpl metric_service;
  duna::FilterServicesImpl filter_service;
  duna::ReconstructServiceImpl reconstruct_service(use_gpu);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&metric_service);
  builder.RegisterService(&filter_service);
  builder.RegisterService(&reconstruct_service);

  std::unique_ptr<Server> server(builder.BuildAndStart());

  std::cout << "Listening to: " << server_address << std::endl;

  server->Wait();

  return 0;
}