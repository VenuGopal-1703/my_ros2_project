from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Absolute path to the URDF file inside the six_dof_arm package
    urdf_file_path = os.path.join(
       #os.path.dirname(__file__), '..', 'src', 'six_dof_arm', 'urdf', 'simple_robot.urdf'
       # 'package://six_dof_arm/urdf/simple_robot.urdf'
        os.path.expanduser('~'), 'ros2_ws3', 'src', 'six_dof_arm', 'urdf', 'simple_robot.sdf '
    )

    # Absolute path to the Gazebo world file
    world_file_path = os.path.join(
        os.path.dirname(__file__), '..', 'worlds', 'empty_world.world'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sdf_file',
            default_value=urdf_file_path,
            description='Path to the SDF file'
        ),
        
        DeclareLaunchArgument(
            'world_file',
            default_value=world_file_path,
            description='Path to the Gazebo world file'
        ),
        
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world_file')],
        ),
        
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzclient',
            name='gazebo_client',
            output='screen',
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # parameters=[{'robot_description': open(urdf_file_path).read()}],
            parameters=[{'robot_description': LaunchConfiguration('urdf_file')}],
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'simple_robot',
                '-file', LaunchConfiguration('sdf_file'),
                 '--ros-args', '-r', '__node:=spawn_entity'
            ],
        ),
    ])
