from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('simplebot_description')
    
    slamtoolbox_param = os.path.join(pkg_dir, 'config', 'slam_config.yaml')
    slamtoolbox_rviz_config = os.path.join(pkg_dir, 'rviz', 'slamtoolbox.rviz')
    
    slam_node = Node(
        package= "slam_toolbox",
        executable= "async_slam_toolbox_node",
        name= "slam_toolbox",
        output="screen",
        parameters=[slamtoolbox_param]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", slamtoolbox_rviz_config],
        output="screen"
    )
    return LaunchDescription([
        slam_node,
        rviz_node
    ])