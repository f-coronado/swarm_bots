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
* @file walk_algorithm.cpp
* @author f-coronado
* @brief Walkser script
* @date 11/26/2023
*
* @copyright Copyright (c) 2023
*
*/

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

#include "my_dummy_lib_funct2.hpp"
#include "path.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

using TWIST     = geometry_msgs::msg::Twist;
using STRING    = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<TWIST>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

class control : public rclcpp::Node {
 public:

  control()
    : Node("control"),
    count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing control node");

    Path path;

    // creates publisher with buffer size of 10
    publisher_ = this->create_publisher<TWIST>("cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Publisher loaded successfully");


    // creates 2 hz timer and ties the callback function
    timer_ =
      this->create_wall_timer(
        500ms,
        std::bind(&control::timer_callback, this));
  }

 private:

  size_t    count_;
  PUBLISHER publisher_;
  TIMER     timer_;
  int nodes_;
  std::vector<int> path_velocities; // update!!


  void timer_callback()
  {
    // // Create the message to publish
    // auto message = TWIST();

    // message.data = "Hello, world! " + std::to_TWIST(count_++);
    // RCLCPP_INFO_STREAM (this->get_logger(),
    //                     "Publishing: " << function2 (count_) << " " << message.data.c_str());

    // // Publish the message
    // publisher_->publish(message);
  }

  void start_path()
  {

    

  }

  void stop_path()
  {

    auto message = TWIST();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_ -> publish(message);
    RCLCPP_INFO(this->get_logger(), "Stopping the robots");

  }


};

int main(int argc, char *argv[])
{
  // 1.) Initialize ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // 2.) Start processing
  rclcpp::spin(std::make_shared<control>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
