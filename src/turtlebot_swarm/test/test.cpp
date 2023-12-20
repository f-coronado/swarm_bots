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

    ASSERT_NO_THROW({
        publisher.publishMessages();
    });
}
