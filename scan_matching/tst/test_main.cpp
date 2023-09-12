#include <gtest/gtest.h>

bool g_visualize = false;

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Check -v option on the first argument only.. getopt does not work well with gtest.
  if (argc > 1) {
    const auto opt = std::string(argv[1]);
    if (opt == "-v") {
      std::cout << "Visualization turned on!\n";
      g_visualize = true;
    }
  }

  return RUN_ALL_TESTS();
}