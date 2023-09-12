#pragma once

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "tools_interface.grpc.pb.h"

class MetricServicesImpl : public PointCloudTools::MetricServices::Service {};
