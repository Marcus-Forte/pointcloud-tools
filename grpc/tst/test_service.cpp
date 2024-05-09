#include <filter_services.h>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

#include <filesystem>

#include "filters.pb.h"

using namespace duna;

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

TEST_F(TestFilterService, testInvalidOutputName) {
  request.add_parameters(1.5);
  request.set_output_name("");

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "Invalid output name.");
}

TEST_F(TestFilterService, testVoxelGridNotEnoughParameters) {
  // Request parameters is zero.
  request.set_input_file("input");
  request.set_output_name("output");
  request.set_operation(PointCloudTools::FilterOperation::VOXEL_GRID);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "Voxel grid takes a single parameter.");
}

TEST_F(TestFilterService, testVoxelGridNegativeResolution) {
  request.add_parameters(-1.5);
  request.set_output_name("output");
  request.set_input_file("input");
  request.set_operation(PointCloudTools::FilterOperation::VOXEL_GRID);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "Voxel grid does not take negative resolution.");
}

TEST_F(TestFilterService, testVoxelGridOK) {
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

TEST_F(TestFilterService, testSORNotEnoughParameters) {
  // Request parameters is zero.
  request.set_input_file("input");
  request.set_output_name("output");
  request.set_operation(PointCloudTools::FilterOperation::SOR);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "SOR takes 2 parameters.");
}

TEST_F(TestFilterService, testSORNegativeStandardDeviation) {
  request.add_parameters(-1.5);
  request.add_parameters(1);
  request.set_output_name("output");
  request.set_input_file("input");
  request.set_operation(PointCloudTools::FilterOperation::SOR);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "SOR does not take negative standard deviation.");
}

TEST_F(TestFilterService, testSORNegativeNeighborsLimit) {
  request.add_parameters(1);
  request.add_parameters(-1.5);
  request.set_output_name("output");
  request.set_input_file("input");
  request.set_operation(PointCloudTools::FilterOperation::SOR);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "SOR does not take negative neighbors limit.");
}

TEST_F(TestFilterService, testSOROK) {
  request.add_parameters(1);
  request.add_parameters(50);
  request.set_output_name("output");

  request.set_operation(PointCloudTools::FilterOperation::SOR);
  auto input_file = std::filesystem::path(TEST_DATA_PATH) / "input_las.las";
  auto output_file = std::filesystem::path(TEST_DATA_PATH) / "output.las";
  request.set_input_file(input_file);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::OK);
  EXPECT_TRUE(std::filesystem::is_regular_file(output_file));
  std::filesystem::remove(output_file);
}

TEST_F(TestFilterService, testRORNotEnoughParameters) {
  // Request parameters is zero.
  request.set_input_file("input");
  request.set_output_name("output");
  request.set_operation(PointCloudTools::FilterOperation::ROR);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "ROR takes 2 parameters.");
}

TEST_F(TestFilterService, testRORNegativeSearchRadius) {
  request.add_parameters(-1.5);
  request.add_parameters(1);
  request.set_output_name("output");
  request.set_input_file("input");
  request.set_operation(PointCloudTools::FilterOperation::ROR);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "ROR does not take negative search radius.");
}

TEST_F(TestFilterService, testRORNegativeNeighborsLimit) {
  request.add_parameters(1);
  request.add_parameters(-1.5);
  request.set_output_name("output");
  request.set_input_file("input");
  request.set_operation(PointCloudTools::FilterOperation::ROR);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(status.error_message(), "ROR does not take negative neighbors limit.");
}

TEST_F(TestFilterService, testROROK) {
  request.add_parameters(0.8);
  request.add_parameters(2);
  request.set_output_name("output");

  request.set_operation(PointCloudTools::FilterOperation::ROR);
  auto input_file = std::filesystem::path(TEST_DATA_PATH) / "input_las.las";
  auto output_file = std::filesystem::path(TEST_DATA_PATH) / "output.las";
  request.set_input_file(input_file);

  auto status = service.applySubsetFilter(mock_context, &request, &response);

  EXPECT_EQ(status.error_code(), ::grpc::StatusCode::OK);
  EXPECT_TRUE(std::filesystem::is_regular_file(output_file));
  std::filesystem::remove(output_file);
}