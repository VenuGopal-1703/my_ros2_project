from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[{'robot_description': 'robot_description'}],
            remappings=[('/robot_description', '/robot_description')]
        ),
        Node(
            package='moveit_ros_planning_interface',
            executable='moveit_planning_interface',
            name='moveit_planning_interface',
            output='screen'
        )
    ])
