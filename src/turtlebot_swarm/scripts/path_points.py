#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist

def publish_messages(node, robot_index):
    publisher = node.create_publisher(Twist, f'/robot_{robot_index}/cmd_vel', 10)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0

    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info(f'Published message for robot_{robot_index}')
        rclpy.spin_once(node)

def main():
    rclpy.init()
    num_robots = input('Enter number of robots: ')
    node = rclpy.create_node('multi_robot_publisher')

    publishers = []
    for i in range(int(num_robots)+1):  # Assuming robots indices from 0 to 5
        publishers.append(node.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10))

    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.0

    while rclpy.ok():
        for i in range(6):
            publishers[i].publish(msg)
            node.get_logger().info(f'Published message for robot_{i}')
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()