#ifndef MULTI_ROBOT_PUBLISHER_HPP
#define MULTI_ROBOT_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MultiRobotPublisher {
public:
    MultiRobotPublisher(int num_robots);
    ~MultiRobotPublisher();

    void publishMessages();

private:
    rclcpp::Node::SharedPtr node;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers;
    std::shared_ptr<geometry_msgs::msg::Twist> message;
    int num_robots;
};

#endif // MULTI_ROBOT_PUBLISHER_HPP