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
 * @file multi_robot_publisher.hpp
 * @author 1412kauti, f-coronado
 * @brief multi robot publisher header file
 * @date 11/26/2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MULTI_ROBOT_PUBLISHER_HPP
#define MULTI_ROBOT_PUBLISHER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class MultiRobotPublisher {
 public:
  /**
   * @brief Used to initialize a node named multi_robot_publisher
   */
  MultiRobotPublisher(int num_robots);
  ~MultiRobotPublisher();

  int getNumRobots() const { return num_robots; }

  void publishMessages();

 private:
  /**< used for creating the MultiRobotPublisher node */
  rclcpp::Node::SharedPtr node;
  /**< used for creating pubslihers */
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr>
      publishers;
  /**< used for constructing twist messge to publish */
  std::shared_ptr<geometry_msgs::msg::Twist> message;
  int num_robots;
};

#endif  // MULTI_ROBOT_PUBLISHER_HPP