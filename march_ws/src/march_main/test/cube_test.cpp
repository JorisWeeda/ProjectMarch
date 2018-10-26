// Copyright 2018 Project March.

#include <gmock/gmock.h>
#include "../src/march_main_node/Cube.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

class cube_test : public ::testing::Test {
 protected:
  TestCube test_cube = TestCube(10);
};

TEST_F(cube_test, TestNothing) {
  ASSERT_EQ(3, 3);
}

TEST_F(cube_test, TestVolume) {
  ASSERT_EQ(10, test_cube.getVolume());
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "testnode");
  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
