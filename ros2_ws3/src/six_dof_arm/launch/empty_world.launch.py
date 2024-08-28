from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so', '-r', 'worlds/empty.world']
        ),
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzclient',
            name='gazebo_client',
            output='screen'
        ),
    ])
