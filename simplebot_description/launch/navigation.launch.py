from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("simplebot_description")
    nav_param_file = os.path.join(pkg_dir, 'config', 'nav2.yaml')
    map_file = os.path.join(pkg_dir, 'map', 'grid_map.yaml')
    
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            'use_sim_time': "true",
            'params_file': nav_param_file,
            'map': map_file,
            'initial_pose': "{'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}"
        }.items()
    )
    
    return LaunchDescription([
        navigation_node
    ])