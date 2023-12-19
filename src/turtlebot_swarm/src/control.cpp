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
#include <cmath>

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

    // int matrixSize;
    // double thetaStep;
    // int rotations;

    // Path pathInst(matrixSize, thetaStep, rotations);

    // std::vector<Path::Robot> robots = pathInst.createRobots(numEnvs);
    // std::vector<Path::Robot> initialPositions = pathInst.startPositions(numEnvs);

    // for (size_t i = 0; i < robots.size(); ++i) {
    //     robots[i] = initialPositions[i];
    // }

    // std::vector<std::pair<double, double>> path_pts = pathInst.pathPoints(circleRad, numEnvs);
    // std::vector<std::vector<std::pair<double, double>>> rotatedPaths = pathInst.trajectoryDict(path_pts);
    // std::cout << "rotatedPaths: " << std::endl;
    // std::cout << "position[3], orientation[3] " << std::endl;

    // for (size_t i = 0; i < rotatedPaths.size(); ++i) {
    //     std::cout << "Robot " << i << ": ";
    //     for (size_t j = 0; j < rotatedPaths[i].size(); ++j) {
    //         std::cout << "(" << rotatedPaths[i][j].first << ", " << rotatedPaths[i][j].second << ") ";
    //     }
    //     std::cout << std::endl;
    // }


    std::vector<std::vector<std::pair<double, double>>> path_points = {
        {{19.9956, 0.418848}, {19.9825, 0.837513}, {19.9605, 1.25581}, {19.9299, 1.67356}, {19.8904, 2.09057}, {19.8423, 2.50666}, {19.7854, 2.92166}, {19.7199, 3.33537}, {19.6457, 3.74763}, {19.563, 4.15823}, {19.4716, 4.56702}, {19.3717, 4.9738}, {19.2633, 5.3784}, {19.1464, 5.78064}, {19.0211, 6.18034}, {18.8875, 6.57733}, {18.7456, 6.97144}, {18.5955, 7.36249}, {18.4373, 7.75031}, {18.2709, 8.13473}, {18.0965, 8.51559}, {17.9142, 8.8927}, {17.7241, 9.26592}, {17.5261, 9.63507}, {17.3205, 10}, {17.1073, 10.3605}, {16.8866, 10.7165}, {16.6584, 11.0678}, {16.423, 11.4143}, {16.1803, 11.7557}, {15.9306, 12.092}, {15.6739, 12.423}, {15.4103, 12.7485}, {15.1399, 13.0684}, {14.8629, 13.3826}, {14.5794, 13.6909}, {14.2895, 13.9933}, {13.9933, 14.2895}, {13.6909, 14.5794}, {13.3826, 14.8629}, {13.0684, 15.1399}, {12.7485, 15.4103}, {12.423, 15.6739}, {12.092, 15.9306}, {11.7557, 16.1803}, {11.4143, 16.423}, {11.0678, 16.6584}, {10.7165, 16.8866}, {10.3605, 17.1073}, {10, 17.3205}, {9.63507, 17.5261}, {9.26592, 17.7241}, {8.8927, 17.9142}, {8.51559, 18.0965}, {8.13473, 18.2709}, {7.75031, 18.4373}, {7.36249, 18.5955}, {6.97144, 18.7456}, {6.57733, 18.8875}, {6.18034, 19.0211}, {5.78064, 19.1464}, {5.3784, 19.2633}, {4.9738, 19.3717}, {4.56702, 19.4716}, {4.15823, 19.563}, {3.74763, 19.6457}, {3.33537, 19.7199}, {2.92166, 19.7854}, {2.50666, 19.8423}, {2.09057, 19.8904}, {1.67356, 19.9299}, {1.25581, 19.9605}, {0.837513, 19.9825}, {0.418848, 19.9956}, {5.66554e-15, 20}, {-0.418848, 19.9956}, {-0.837513, 19.9825}, {-1.25581, 19.9605}, {-1.67356, 19.9299}, {-2.09057, 19.8904}, {-2.50666, 19.8423}, {-2.92166, 19.7854}, {-3.33537, 19.7199}, {-3.74763, 19.6457}, {-4.15823, 19.563}, {-4.56702, 19.4716}, {-4.9738, 19.3717}, {-5.3784, 19.2633}, {-5.78064, 19.1464}, {-6.18034, 19.0211}, {-6.57733, 18.8875}, {-6.97144, 18.7456}, {-7.36249, 18.5955}, {-7.75031, 18.4373}, {-8.13473, 18.2709}, {-8.51559, 18.0965}, {-8.8927, 17.9142}, {-9.26592, 17.7241}, {-9.63507, 17.5261}, {-10, 17.3205}, {-10.3605, 17.1073}, {-10.7165, 16.8866}, {-11.0678, 16.6584}, {-11.4143, 16.423}, {-11.7557, 16.1803}, {-12.092, 15.9306}, {-12.423, 15.6739}, {-12.7485, 15.4103}, {-13.0684, 15.1399}, {-13.3826, 14.8629}, {-13.6909, 14.5794}, {-13.9933, 14.2895}, {-14.2895, 13.9933}, {-14.5794, 13.6909}, {-14.8629, 13.3826}, {-15.1399, 13.0684}, {-15.4103, 12.7485}, {-15.6739, 12.423}, {-15.9306, 12.092}, {-16.1803, 11.7557}, {-16.423, 11.4143}, {-16.6584, 11.0678}, {-16.8866, 10.7165}, {-17.1073, 10.3605}, {-17.3205, 10}, {-17.5261, 9.63507}, {-17.7241, 9.26592}, {-17.9142, 8.8927}, {-18.0965, 8.51559}, {-18.2709, 8.13473}, {-18.4373, 7.75031}, {-18.5955, 7.36249}, {-18.7456, 6.97144}, {-18.8875, 6.57733}, {-19.0211, 6.18034}, {-19.1464, 5.78064}, {-19.2633, 5.3784}, {-19.3717, 4.9738}, {-19.4716, 4.56702}, {-19.563, 4.15823}, {-19.6457, 3.74763}, {-19.7199, 3.33537}, {-19.7854, 2.92166}, {-19.8423, 2.50666}, {-19.8904, 2.09057}, {-19.9299, 1.67356}, {-19.9605, 1.25581}, {-19.9825, 0.837513}, {-19.9956, 0.418848}, {-20, 1.13311e-14}, {-19.9956, -0.418848}, {-19.9825, -0.837513}, {-19.9605, -1.25581}, {-19.9299, -1.67356}, {-19.8904, -2.09057}, {-19.8423, -2.50666}, {-19.7854, -2.92166}, {-19.7199, -3.33537}, {-19.6457, -3.74763}, {-19.563, -4.15823}, {-19.4716, -4.56702}, {-19.3717, -4.9738}, {-19.2633, -5.3784}, {-19.1464, -5.78064}, {-19.0211, -6.18034}, {-18.8875, -6.57733}, {-18.7456, -6.97144}, {-18.5955, -7.36249}, {-18.4373, -7.75031}, {-18.2709, -8.13473}, {-18.0965, -8.51559}, {-17.9142, -8.8927}, {-17.7241, -9.26592}, {-17.5261, -9.63507}, {-17.3205, -10}, {-17.1073, -10.3605}, {-16.8866, -10.7165}, {-16.6584, -11.0678}, {-16.423, -11.4143}, {-16.1803, -11.7557}, {-15.9306, -12.092}, {-15.6739, -12.423}, {-15.4103, -12.7485}, {-15.1399, -13.0684}, {-14.8629, -13.3826}, {-14.5794, -13.6909}, {-14.2895, -13.9933}, {-13.9933, -14.2895}, {-13.6909, -14.5794}, {-13.3826, -14.8629}, {-13.0684, -15.1399}, {-12.7485, -15.4103}, {-12.423, -15.6739}, {-12.092, -15.9306}, {-11.7557, -16.1803}, {-11.4143, -16.423}, {-11.0678, -16.6584}, {-10.7165, -16.8866}, {-10.3605, -17.1073}, {-10, -17.3205}, {-9.63507, -17.5261}, {-9.26592, -17.7241}, {-8.8927, -17.9142}, {-8.51559, -18.0965}, {-8.13473, -18.2709}, {-7.75031, -18.4373}, {-7.36249, -18.5955}, {-6.97144, -18.7456}, {-6.57733, -18.8875}, {-6.18034, -19.0211}, {-5.78064, -19.1464}, {-5.3784, -19.2633}, {-4.9738, -19.3717}, {-4.56702, -19.4716}, {-4.15823, -19.563}, {-3.74763, -19.6457}, {-3.33537, -19.7199}, {-2.92166, -19.7854}, {-2.50666, -19.8423}, {-2.09057, -19.8904}, {-1.67356, -19.9299}, {-1.25581, -19.9605}, {-0.837513, -19.9825}, {-0.418848, -19.9956}, {-3.67394e-15, -20}, {0.418848, -19.9956}, {0.837513, -19.9825}, {1.25581, -19.9605}, {1.67356, -19.9299}, {2.09057, -19.8904}, {2.50666, -19.8423}, {2.92166, -19.7854}, {3.33537, -19.7199}, {3.74763, -19.6457}, {4.15823, -19.563}, {4.56702, -19.4716}, {4.9738, -19.3717}, {5.3784, -19.2633}, {5.78064, -19.1464}, {6.18034, -19.0211}, {6.57733, -18.8875}, {6.97144, -18.7456}, {7.36249, -18.5955}, {7.75031, -18.4373}, {8.13473, -18.2709}, {8.51559, -18.0965}, {8.8927, -17.9142}, {9.26592, -17.7241}, {9.63507, -17.5261}, {10, -17.3205}, {10.3605, -17.1073}, {10.7165, -16.8866}, {11.0678, -16.6584}, {11.4143, -16.423}, {11.7557, -16.1803}, {12.092, -15.9306}, {12.423, -15.6739}, {12.7485, -15.4103}, {13.0684, -15.1399}, {13.3826, -14.8629}, {13.6909, -14.5794}, {13.9933, -14.2895}, {14.2895, -13.9933}, {14.5794, -13.6909}, {14.8629, -13.3826}, {15.1399, -13.0684}, {15.4103, -12.7485}, {15.6739, -12.423}, {15.9306, -12.092}, {16.1803, -11.7557}, {16.423, -11.4143}, {16.6584, -11.0678}, {16.8866, -10.7165}, {17.1073, -10.3605}, {17.3205, -10}, {17.5261, -9.63507}, {17.7241, -9.26592}, {17.9142, -8.8927}, {18.0965, -8.51559}, {18.2709, -8.13473}, {18.4373, -7.75031}, {18.5955, -7.36249}, {18.7456, -6.97144}, {18.8875, -6.57733}, {19.0211, -6.18034}, {19.1464, -5.78064}, {19.2633, -5.3784}, {19.3717, -4.9738}, {19.4716, -4.56702}, {19.563, -4.15823}, {19.6457, -3.74763}, {19.7199, -3.33537}, {19.7854, -2.92166}, {19.8423, -2.50666}, {19.8904, -2.09057}, {19.9299, -1.67356}, {19.9605, -1.25581}, {19.9825, -0.837513}, {19.9956, -0.418848}, {20, -2.26622e-14}, {20, 0}}, // Robot 1
        // ... additional trajectories for other robots
    };

    for (size_t i = 0; i < path_points.size(); ++i) {
      std::cout << "Robot " << i << ": ";
      for (size_t j = 0; j < path_points[i].size(); ++j) {
        std::cout << "(" << path_points[i][j].first << ", " << path_points[i][j].second << ") ";
      }
      std::cout << std::endl;
    }
    std::vector<std::pair<double, double>> path_velocities = createVelocities(path_points[0]); 
    for (size_t i = 0; i < path_velocities.size(); ++i) {
        std::cout << "Point " << i + 1 << ": Linear Velocity = " << path_velocities[i].first
                  << ", Angular Velocity = " << path_velocities[i].second << std::endl;
    }
    start_path(path_velocities, 1);

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
  std::vector<std::vector<double>> path_velocities; // update!!
  std::vector<std::vector<std::pair<double, double>>> path_points;
  double timeStep = 1.0;



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
  void start_path(const std::vector<std::pair<double, double>>& path_velocities, int robot_id) {
      RCLCPP_INFO(this->get_logger(), "Starting the path for robot %d", robot_id);

      // Construct the namespace for the robot
      std::string robot_namespace = "/robot_" + std::to_string(robot_id);

      // Construct the full topic names using the robot's namespace
      std::string cmd_vel_topic = robot_namespace + "/cmd_vel";

      for (size_t i = 0; i < path_velocities.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "Moving to point %zu for the robots \n");

          // Construct twist message to publish to the cmd_vel topic
          auto cmd_vel_message = TWIST();
          cmd_vel_message.linear.x = path_velocities[i].first/10;
          cmd_vel_message.angular.z = path_velocities[i].second/10;

          // Publish to cmd_vel topic
          auto cmd_vel_publisher = this->create_publisher<TWIST>(cmd_vel_topic, 10);
          cmd_vel_publisher->publish(cmd_vel_message);

          auto cmd_vel_publisher_1 = this->create_publisher<TWIST>("/robot_1/cmd_vel", 10);
          auto cmd_vel_publisher_2 = this->create_publisher<TWIST>("/robot_2/cmd_vel", 10);
          auto cmd_vel_publisher_3 = this->create_publisher<TWIST>("/robot_3/cmd_vel", 10);
          auto cmd_vel_publisher_4 = this->create_publisher<TWIST>("/robot_4/cmd_vel", 10);
          auto cmd_vel_publisher_5 = this->create_publisher<TWIST>("/robot_5/cmd_vel", 10);
          auto cmd_vel_publisher_6 = this->create_publisher<TWIST>("/robot_6/cmd_vel", 10);
          auto cmd_vel_publisher_7 = this->create_publisher<TWIST>("/robot_7/cmd_vel", 10);
          auto cmd_vel_publisher_8 = this->create_publisher<TWIST>("/robot_8/cmd_vel", 10);
          auto cmd_vel_publisher_9 = this->create_publisher<TWIST>("/robot_9/cmd_vel", 10);
          auto cmd_vel_publisher_10 = this->create_publisher<TWIST>("/robot_10/cmd_vel", 10);
          auto cmd_vel_publisher_11 = this->create_publisher<TWIST>("/robot_11/cmd_vel", 10);
          auto cmd_vel_publisher_12 = this->create_publisher<TWIST>("/robot_12/cmd_vel", 10);
          auto cmd_vel_publisher_13 = this->create_publisher<TWIST>("/robot_13/cmd_vel", 10);
          auto cmd_vel_publisher_14 = this->create_publisher<TWIST>("/robot_14/cmd_vel", 10);
          auto cmd_vel_publisher_15 = this->create_publisher<TWIST>("/robot_15/cmd_vel", 10);
          auto cmd_vel_publisher_16 = this->create_publisher<TWIST>("/robot_16/cmd_vel", 10);
          auto cmd_vel_publisher_17 = this->create_publisher<TWIST>("/robot_17/cmd_vel", 10);
          auto cmd_vel_publisher_18 = this->create_publisher<TWIST>("/robot_18/cmd_vel", 10);
          auto cmd_vel_publisher_19 = this->create_publisher<TWIST>("/robot_19/cmd_vel", 10);
          
          cmd_vel_publisher_1->publish(cmd_vel_message);
          cmd_vel_publisher_2->publish(cmd_vel_message);
          cmd_vel_publisher_3->publish(cmd_vel_message);
          cmd_vel_publisher_4->publish(cmd_vel_message);
          cmd_vel_publisher_5->publish(cmd_vel_message);
          cmd_vel_publisher_6->publish(cmd_vel_message);
          cmd_vel_publisher_7->publish(cmd_vel_message);
          cmd_vel_publisher_8->publish(cmd_vel_message);
          cmd_vel_publisher_9->publish(cmd_vel_message);
          cmd_vel_publisher_10->publish(cmd_vel_message);
          cmd_vel_publisher_11->publish(cmd_vel_message);
          cmd_vel_publisher_12->publish(cmd_vel_message);
          cmd_vel_publisher_13->publish(cmd_vel_message);
          cmd_vel_publisher_14->publish(cmd_vel_message);
          cmd_vel_publisher_15->publish(cmd_vel_message);
          cmd_vel_publisher_16->publish(cmd_vel_message);
          cmd_vel_publisher_17->publish(cmd_vel_message);
          cmd_vel_publisher_18->publish(cmd_vel_message);
          cmd_vel_publisher_19->publish(cmd_vel_message);



          // You can similarly publish to other topics (imu, joint_states, odom, scan) here

          // Sleep or wait for some time before moving to the next point
          rclcpp::sleep_for(500ms);
      }

      // // Stop the bots after completing the trajectory
      // stop_path(robot_id);
  }



  std::vector<std::pair<double, double>> createVelocities(const std::vector<std::pair<double, double>>& points) {
      // Assuming a constant time step, you might need to adjust this based on your application
      const double timeStep = 0.1;

      std::vector<std::pair<double, double>> velocities;

      for (size_t i = 1; i < points.size(); ++i) {
          // Calculate the differences in x and y
          double dx = points[i].first - points[i - 1].first;
          double dy = points[i].second - points[i - 1].second;

          // Calculate linear velocity (magnitude of the velocity vector)
          double segmentLinearVelocity = std::sqrt(dx * dx + dy * dy) / timeStep;

          // Calculate angular velocity (yaw component)
          double segmentAngularVelocity = std::atan2(dy, dx) / timeStep;

          velocities.push_back(std::make_pair(segmentLinearVelocity, segmentAngularVelocity));
      }

      return velocities;
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
