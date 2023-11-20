#include <filter_services.h>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

#include <filesystem>

#include "filters.pb.h"

class TestFilterService : public ::testing::Test {
 public:
  TestFilterService() {
    mock_context = new ::grpc::ServerContext;

    request.set_operation(PointCloudTools::FilterOperation::VOXEL_GRID);
  }

  virtual ~TestFilterService() { delete mock_context; }

 protected:
  ::grpc::ServerContext* mock_context;
  FilterServicesImpl service;
  PointCloudTools::subsetFilterRequest request;
  PointCloudTools::stringResponse response;
};

TEST_F(TestFilterService, testNotEnoughParameters) {
  // Request parameters is zero.
  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "Voxel grid takes a single parameter.");
}

TEST_F(TestFilterService, testNegativeResolution) {
  request.add_parameters(-1.5);
  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "Voxel grid does not take negative resolution.");
}

TEST_F(TestFilterService, testInvalidOutputName) {
  request.add_parameters(1.5);
  request.set_output_name("");

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "Invalid output name.");
}

TEST_F(TestFilterService, voxelGridOK) {
  request.add_parameters(50);
  request.set_output_name("output");
  request.set_operation(PointCloudTools::FilterOperation::VOXEL_GRID);
  auto input_file = std::filesystem::path(TEST_DATA_PATH) / "input_las.las";
  auto output_file = std::filesystem::path(TEST_DATA_PATH) / "output.las";
  request.set_input_file(input_file);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::OK);
  EXPECT_TRUE(std::filesystem::is_regular_file(output_file));
  std::filesystem::remove(output_file);
}