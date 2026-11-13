#include <gtest/gtest.h>
#include <hal/HAL.h>

int main(int argc, char** argv) {
  if (!HAL_Initialize(500, 0)) {
    return 1;
  }
  // Native Gradle tests run in a separate directory per build variant.
  // Keep a reviewable XML report there; command-line flags may override it.
  GTEST_FLAG_SET(output, "xml:test-results.xml");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
