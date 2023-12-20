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
* @file multi_robot_publisher.cpp
* @author 1412kauti, f-coronado
* @brief multi robot publisher
* @date 11/26/2023
*
* @copyright Copyright (c) 2023
*
*/

#include "swarm_library/multi_robot_publisher.hpp"


/**
* @brief Create the multi robot publisher node
*/
MultiRobotPublisher::MultiRobotPublisher(int num_robots) : num_robots(num_robots) {
    rclcpp::init(0, nullptr);


    /**
    * @brief create a number of publishers dependent on number of bots
    */
    node = std::make_shared<rclcpp::Node>("multi_robot_publisher");


    /**
    * @brief Create aa vector of publishers
    */
    for (int i = 0; i < num_robots; ++i) {
        publishers.push_back(node->create_publisher<geometry_msgs::msg::Twist>(
            "/robot_" + std::to_string(i) + "/cmd_vel", 10));
    }

    /**
    * @brief construct the message to publish 
    */
    message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = 0.1;
    message->angular.z = 0.0;
}

MultiRobotPublisher::~MultiRobotPublisher() {
    rclcpp::shutdown();
}

/**
* @brief Iterates over publisher vector and publishes message
*/
void MultiRobotPublisher::publishMessages() {
    while (rclcpp::ok()) {
        for (int i = 0; i < num_robots; ++i) {
            publishers[i]->publish(*message);
            RCLCPP_INFO(node->get_logger(), "Published message for robot_%d", i);
        }
        rclcpp::spin_some(node);
    }
}