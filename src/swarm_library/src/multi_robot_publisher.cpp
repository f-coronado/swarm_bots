#include "swarm_library/multi_robot_publisher.hpp"

MultiRobotPublisher::MultiRobotPublisher(int num_robots) : num_robots(num_robots) {
    rclcpp::init(0, nullptr);

    node = std::make_shared<rclcpp::Node>("multi_robot_publisher");

    for (int i = 0; i < num_robots; ++i) {
        publishers.push_back(node->create_publisher<geometry_msgs::msg::Twist>(
            "/robot_" + std::to_string(i) + "/cmd_vel", 10));
    }

    message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = 0.1;
    message->angular.z = 0.0;
}

MultiRobotPublisher::~MultiRobotPublisher() {
    rclcpp::shutdown();
}

void MultiRobotPublisher::publishMessages() {
    while (rclcpp::ok()) {
        for (int i = 0; i < num_robots; ++i) {
            publishers[i]->publish(*message);
            RCLCPP_INFO(node->get_logger(), "Published message for robot_%d", i);
        }
        rclcpp::spin_some(node);
    }
}