from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pc2_to_scan')

    # 기본 config 경로(설치된 share 기준)
    default_params = os.path.join(pkg_share, 'config', 'pc2_to_scan.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the ROS2 parameters YAML file'
        ),

        Node(
            package='pc2_to_scan',
            executable='pointcloud_to_laserscan_zfilter',
            name='pointcloud_to_laserscan_zfilter',
            output='screen',
            parameters=[params_file],
        ),
    ])
