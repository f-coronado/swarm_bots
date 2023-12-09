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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "path.hpp"

using std::placeholders::_1;
using std_msgs::msg::String;

using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;

using STRING_MSG = std_msgs::msg::String;

class MinimalSubscriber : public rclcpp::Node {
public:

  MinimalSubscriber(
    const std::string& node_name      = "my_node",
    const std::string& node_namespace = "/my_ns",
    const std::string& topic_name     = "my_topic")
    : Node(node_name, node_namespace)
  {
    for (int idx = 0; idx < numSubs; idx++)
    {
      std::string subName = "subscription" + std::to_string(idx);
      std::function<void(const STRING_MSG& msg)> callback =
        std::bind(&MinimalSubscriber::topic_callback, this, _1, subName);
      subscriptions_[idx] = this->create_subscription<String>(
        topic_name,
        10,
        callback);
    }

    function1 (23);             // test model library
  }

private:

  void topic_callback(const STRING_MSG& msg, std::string subName)
  {
    RCLCPP_INFO (this->get_logger(), "subName=%s, I heard : '%s'",
                 subName.c_str(), msg.data.c_str());
  }

  int numSubs                           = 5;
  std::vector<SUBSCRIBER>subscriptions_ = std::vector<SUBSCRIBER>(numSubs);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>("Listen_Node", "/", "topic"));
  rclcpp::shutdown();

  return 0;
}
