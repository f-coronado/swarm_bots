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
* @file control.cpp
* @author f-coronado
* @brief Control script
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

#include "path.hpp"

using namespace std::chrono_literals;

using TWIST     = geometry_msgs::msg::Twist;
using STRING    = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<TWIST>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

class control : public rclcpp::Node {
 public:

  /**
  * @brief Control node which will be used to manipulate bots
  */
  control()
    : Node("control"),
    count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing control node");
    struct Robot {
        double position[3];
        double orientation[3];
    };

    int numEnvs, circleRad;
    std::cout << "Enter the number of nodes: ";
    std::cin >> numEnvs;
    std::cout << "Enter the radius of the circle: ";
    std::cin >> circleRad;

    int matrixSize;
    double thetaStep;
    int rotations;

    Path pathInst(matrixSize, thetaStep, rotations);

    std::vector<Path::Robot> robots = pathInst.createRobots(numEnvs);
    std::vector<Path::Robot> initialPositions = pathInst.startPositions(numEnvs);

    for (size_t i = 0; i < robots.size(); ++i) {
        robots[i] = initialPositions[i];
    }

    std::vector<std::pair<double, double>> path_pts = pathInst.pathPoints(circleRad, numEnvs);
    std::vector<std::vector<std::pair<double, double>>> rotatedPaths = pathInst.trajectoryDict(path_pts);
    std::cout << "rotatedPaths: " << std::endl;
    std::cout << "position[3], orientation[3] " << std::endl;

    for (size_t i = 0; i < rotatedPaths.size(); ++i) {
        std::cout << "Robot " << i << ": ";
        for (size_t j = 0; j < rotatedPaths[i].size(); ++j) {
            std::cout << "(" << rotatedPaths[i][j].first << ", " << rotatedPaths[i][j].second << ") ";
        }
        std::cout << std::endl;
    }


    this->nodes_ = nodes_;
    // this->agents = agents;

    // creates publisher with buffer size of 10
    publisher_ = this->create_publisher<TWIST>("cmd_vel", 10);

    // creates 2 hz timer and ties the callback function
    timer_ =
      this->create_wall_timer(
        500ms,
        std::bind(&control::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Publisher loaded successfully");
  }

 private:

  /**
  * @brief Private members needed for control node and other methods
  */
  size_t    count_;
  PUBLISHER publisher_;
  TIMER     timer_;
  int       nodes_;
  // std::vector<std::shared_ptr<Agent>> &agents;
  std::vector<int> path_velocities; // update!!

  /**
  * @brief Used by control node to update node and tigger any methods
  *
  * @param response_future
  */
  void timer_callback()
  {

  }

  /**
  * @brief starts moving all the bots
  */
  void start_path()
  {

    RCLCPP_INFO(this->get_logger(), "Starting the path");
    int num = 0;
    // for (float i = 0 i < )

    // for (int i = 0; i < path_velocities.size(); ){
      // RCLCPP_INFO(this->get_logger(), "Moving in path");
      
      // construct twist message to publish to bots
      // auto message = TWIST();
      // message.linear.x = path_velocities[i][0];
      // message.angular.z = path_velocities[i][0];
      // publisher_ -> publish(message);

  }

  /**
  * @brief stops moving all the bots
  */
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

  // std::vector<std::shared_ptr<Agent>>
  int nodes_ = 20;
  int count_ = 0;

  // 2.) Start processing
  rclcpp::spin(std::make_shared<control>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
