import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map2gazebo_dir = get_package_share_directory('map2gazebo')
    default_params_file = os.path.join(map2gazebo_dir, 'config', 'defaults.yaml')
    default_export_dir = os.path.join(map2gazebo_dir, 'models', 'map', 'meshes')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the parameters file to load'
        ),
        DeclareLaunchArgument(
            'export_dir',
            default_value=default_export_dir,
            description='Directory to export the models'
        ),

        Node(
            package='map2gazebo',
            executable='map2gazebo.py',
            name='map2gazebo',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            arguments=['--export_dir', LaunchConfiguration('export_dir')],
        ),
    ])

