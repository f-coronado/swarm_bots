#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

void publish_messages(rclcpp::Node::SharedPtr node, int robot_index) {
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "/robot_" + std::to_string(robot_index) + "/cmd_vel", 10);

    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = 0.0;
    message->angular.z = 0.0;

    while (rclcpp::ok()) {
        publisher->publish(*message);
        RCLCPP_INFO(node->get_logger(), "Published message for robot_%d", robot_index);
        rclcpp::spin_some(node);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    int num_robots;
    std::cout << "Enter number of robots: ";
    std::cin >> num_robots;

    auto node = std::make_shared<rclcpp::Node>("multi_robot_publisher");

    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers;
    for (int i = 0; i < num_robots; ++i) {
        publishers.push_back(node->create_publisher<geometry_msgs::msg::Twist>(
            "/robot_" + std::to_string(i) + "/cmd_vel", 10));
    }

    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = 0.1;
    message->angular.z = 0.0;

    while (rclcpp::ok()) {
        for (int i = 0; i < num_robots; ++i) {
            publishers[i]->publish(*message);
            RCLCPP_INFO(node->get_logger(), "Published message for robot_%d", i);
        }
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}