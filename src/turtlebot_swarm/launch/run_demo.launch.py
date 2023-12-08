from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_node = Node(
        package='turtlebot_swarm',
        executable='talker'
    )

    my_node2 = Node(
        package='turtlebot_swarm',
        executable='listener'
    )

    ld.add_action(my_node)
    ld.add_action(my_node2)

    return ld
