"""
@file run_demo.launch.py.

@brief This launch file spawns turtlebots
@date 12/10/2023
@copyright Copyright (c) 2023
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

for userInput in sys.argv:
    if userInput.startswith("node_number:="):
        node_number = int(userInput.split(":=")[1])
    else:
        node_number = 20

def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher_filepath = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch') # issue here

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_state_publisher_filepath, 
            'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    world_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 
    'launch', '/turtlebot3_world.launch.py')

    gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    sdf_file_name = 'model.sdf'
    print('sdf_file_name : {}'.format(sdf_file_name))
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf')

    y_pose = LaunchConfiguration('y_pose', default='0.0')

    for i in range(node_number):
        robot_id = "robot_"+str(i)
        x_val = str(float(i-(node_number/2)))
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_id,
                '-file', urdf,
                '-x', x_val,
                '-y', '0.0',
                '-z', '0.01',
                '-robot_namespace', robot_id
            ],
            output='screen',
        )
        ld.add_action(TimerAction(period=10.0+float(i*2),
                                  actions=[node],))

    # ld.add_action(control_node)
    # ld.add_action(agent_node)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)

    return ld
