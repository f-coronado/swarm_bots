# /******************************************************************************
#  * MIT License
#  *
#  * Copyright (c) 2022 Fabrizzio Coronado
#  *
#  * Permission is hereby granted, free of charge, to any person obtaining a copy
#  * of this software and associated documentation files (the "Software"), to deal
#  * in the Software without restriction, including without limitation the rights
#  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  * copies of the Software, and to permit persons to whom the Software is
#  * furnished to do so, subject to the following conditions:
#  *
#  * The above copyright notice and this permission notice shall be included in
#  * all copies or substantial portions of the Software.
#  *
#  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  * SOFTWARE.
#  ******************************************************************************/

"""
multi_robot_publisher.py
Author: 1412kauti, f-coronado
Date: 11/26/2023
"""

# Import necessary modules
import os
import sys
import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Function to generate circular path points
def path_points(rad, num):
"""
Constructs a list of points to follow

Args:
    rad (double) : the radius of the circle
"""
    radius = rad
    num_points = num
    theta = np.linspace(0, 2 * np.pi, num_points + 1)
    points_list = [(radius * np.cos(angle), radius * np.sin(angle)) for angle in theta]
    return points_list

# Get user inputs for the number of robots and radius
try:
    node_number = int(input("Enter the number of robots: "))
    radius = float(input("Enter the radius for the circular path: "))
except ValueError:
    print("Invalid input. Please enter a valid number.")
    sys.exit(1)

"""
Create description for launch file
"""
def generate_launch_description():
    ld = LaunchDescription()

    # Set launch configuration for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define file paths and directories
    robot_state_publisher_filepath = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )

    world_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', '/turtlebot3_world.launch.py')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # Include Robot State Publisher launch
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_state_publisher_filepath, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include Gazebo server launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Include Gazebo client launch
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Set TurtleBot3 model
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    sdf_file_name = 'model.sdf'
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Loop to spawn TurtleBots along a circular path
    for i in range(node_number):
        robot_id = "robot_" + str(i)
        path = path_points(radius, node_number)
        x_val, y_val = path[i]

        # Create Node for spawning the TurtleBot
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_id,
                '-file', urdf,
                '-x', str(x_val),
                '-y', str(y_val),
                '-z', '0.01',
                '-robot_namespace', robot_id
            ],
            output='screen',
        )

        # Add TimerAction to stagger the spawning of robots
        ld.add_action(TimerAction(period=10.0 + float(i * 2), actions=[node]))

    # Add other actions
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)

    return ld
