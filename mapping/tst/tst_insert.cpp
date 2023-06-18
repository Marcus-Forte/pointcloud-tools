#include <gtest/gtest.h>

#include "../include/voxel_hashing.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST(VoxelHashMap, Insertion) { VoxelHashMap map; }