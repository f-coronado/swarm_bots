/******************************************************************************

* MIT License
*
* Copyright (c) 2022 Fabrizzio Coronado
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.

******************************************************************************/

/**
* @file test.cpp
* @author f-coronado
* @brief Test script
* @date 12/08/2023
*
* @copyright Copyright (c) 2023
*
*/

#include <gtest/gtest.h>
#include "path.hpp"

TEST(PathTest, GetPath) {
  // Create an instance of the path class

  Path path;



  std::vector<std::vector<std::pair<double, double>>> expectedPaths = {

      {{-10, 1.22465e-15}, {10, -2.44929e-15}, {10, 0}},
      {{-10, 1.22465e-15}, {10, -2.44929e-15}, {10, 0}},
      {{-10, 1.22465e-15}, {10, -2.44929e-15}, {10, 0}}
          // ... add more if needed

  };

  // Invoke the function under test
  int numEnvs = 4;
  int circleRad = 4;

  std::vector<Path::Robot> robots = path.createRobots(numEnvs);
  std::vector<Path::Robot> initialPositions = path.startPositions(numEnvs);
  std::vector<std::pair<double, double>> test_path =
      path.pathPoints(circleRad, numEnvs);
  std::vector<std::vector<std::pair<double, double>>> actualPaths =
      path.trajectoryDict(test_path);

  // Use gtest macros to make assertions
  ASSERT_EQ(expectedPaths.size(), actualPaths.size());

  for (size_t i = 0; i < expectedPaths.size(); ++i) {
    ASSERT_EQ(expectedPaths[i].size(), actualPaths[i].size());

    for (size_t j = 0; j < expectedPaths[i].size(); ++j) {
      EXPECT_DOUBLE_EQ(expectedPaths[i][j].first, actualPaths[i][j].first);
      EXPECT_DOUBLE_EQ(expectedPaths[i][j].second, actualPaths[i][j].second);
    }
  }
}
#include <gtest/gtest.h>
#include "swarm_library/multi_robot_publisher.hpp"
TEST(MultiRobotPublisherTest, Constructor) {
    // Test constructor and getNumRobots method
    MultiRobotPublisher publisher(3);
    EXPECT_EQ(publisher.getNumRobots(), 3);
}

TEST(MultiRobotPublisherTest, PublishMessages) {
    // Test the publishMessages method
    MultiRobotPublisher publisher(2);

    // Your implementation may involve creating a ROS 2 executor or a mock node for testing.
    // Here, we assume that the publishMessages method works correctly without actual message publishing.

    // Calling the method should not cause any crashes or errors.
    ASSERT_NO_THROW({
        publisher.publishMessages();
    });
}