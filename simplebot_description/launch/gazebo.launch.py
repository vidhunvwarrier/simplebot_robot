# Launch file for starting Gazebo with the simple bot

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("simplebot_description")
    
    urdf_path = os.path.join(pkg_dir, "urdf", "simple_bot.urdf.xacro")
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    world_path = os.path.join(pkg_dir, "world", "grid.world")
    
    # Gazebo launch file
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )]
        ),
        launch_arguments= {'world': world_path}.items()
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_bot', '-topic', 'robot_description'],
        output='screen'
    )

    # Robot Localization Node
    robot_localization_node = Node(
        package= 'robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo_node,
        robot_state_publisher_node,
        spawn_robot,
        robot_localization_node
    ])
