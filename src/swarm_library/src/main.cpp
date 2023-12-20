#include "swarm_library/multi_robot_publisher.hpp"
#include <iostream>

int main() {
    int num_robots;
    std::cout << "Enter number of robots: ";
    std::cin >> num_robots;

    MultiRobotPublisher multiRobotPublisher(num_robots);
    multiRobotPublisher.publishMessages();

    return 0;
}