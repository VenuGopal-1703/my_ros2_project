from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/venu/ros2_ws3/src/six_dof_arm/urdf/six_dof_arm.urdf', 'r').read()}]
        ),
    ])
