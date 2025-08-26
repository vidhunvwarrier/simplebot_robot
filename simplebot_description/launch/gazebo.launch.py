from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkd_dir = get_package_share_directory("simplebot_description")
    
    urdf_path = os.path.join(pkd_dir, "urdf", "simple_bot.urdf.xacro")
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name= 'joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_bot', '-topic', 'robot_description'],
        output='screen'
    )
    
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_robot
    ])
