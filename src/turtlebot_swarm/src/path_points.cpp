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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

/**
* @file path_points.cpp
* @author 1412kauti, f-coronado
* @brief multi robot publisher
* @date 11/26/2023
*
* @copyright Copyright (c) 2023
*
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
* @brief Iterates over publisher vector and publishes message
* @param node node num
* @param robot_index robot id
*/
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
    
    /**
    * @brief gather user input
    */
    int num_robots;
    std::cout << "Enter number of robots: ";
    std::cin >> num_robots;

    /**
    * @brief create publisher
    */
    auto node = std::make_shared<rclcpp::Node>("multi_robot_publisher");

    /**
    * @brief Create vector of publishers
    */
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers;
    for (int i = 0; i < num_robots; ++i) {
        publishers.push_back(node->create_publisher<geometry_msgs::msg::Twist>(
            "/robot_" + std::to_string(i) + "/cmd_vel", 10));
    }

    /**
    * @brief construct message
    */
    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = 0.1;
    message->angular.z = 0.0;

    /**
    * @brief publish message constructed using vector of publishers
    */
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