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
* @file agent.cpp
* @author f-coronado
* @brief Agent script
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
using SUBSCRIBER = rclcpp::Subscription<TWIST>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

class agent : public rclcpp::Node {
 public:

  /**
  * agent node which will be used to manipulate bots
  */
  agent()
    : Node("agent"),
    count_(0)
  {
    // RCLCPP_INFO(this->get_logger(), "Initializing agent node");

    // Path path;

    // // creates publisher with buffer size of 10
    // subscriber_ = this->create_subscription<TWIST>("cmd_vel", 10);
    // RCLCPP_INFO(this->get_logger(), "Publisher loaded successfully");

    // // creates 2 hz timer and ties the callback function
    // timer_ =
    //   this->create_wall_timer(
    //     500ms,
    //     std::bind(&agent::timer_callback, this));
  }

 private:

  /**
  * @brief Private members needed for agent node and other methods
  */
  size_t    count_;
  SUBSCRIBER subscriber_;
  TIMER     timer_;
  float x_vel;
  float y_vel;
  float z_vel;
  float roll;
  float pitch;
  float yaw;
  int robot_num;

  /**
  * @brief starts moving all the bots
  */
  void move()
  {

    RCLCPP_INFO(this->get_logger(), "Starting the robots");
    
  }

  /**
  * @brief stops moving all the bots
  */
  void stop()
  {

    // auto message = TWIST();
    // message.linear.x = 0.0;
    // message.angular.z = 0.0;
    // subscriber_ -> publish(message);
    RCLCPP_INFO(this->get_logger(), "Stopping the robots");

  }

};

int main(int argc, char *argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agent>());
  rclcpp::shutdown();
  return 0;
}
