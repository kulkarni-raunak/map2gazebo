import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare as FindPackageShareRos

def generate_launch_description():
    # Paths to the package directories
    map2gazebo_share = FindPackageShare('map2gazebo').find('map2gazebo')
    gazebo_ros_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Set gz-sim resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=[
        os.path.join(get_package_prefix('map2gazebo'), "share"),
        ":" +
        os.path.join(get_package_share_directory('map2gazebo'), "models")])
    
    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', TURTLEBOT3_MODEL,
                   '-allow_renaming', 'true',
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )

    gazebo_spawn_world = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('turtlebot3_gazebo'),
                        "models",
                        world_name,
                        "model.sdf"]),
                   '-allow_renaming', 'false'],
        )

    basic_world = os.path.join(get_package_share_directory('turtlebot3_gz'), "worlds", "empty.sdf")
    
    # Define launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false', description='Enable debugging'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable GUI'),
        DeclareLaunchArgument('headless', default_value='false', description='Run in headless mode'),
        
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')),
            launch_arguments={
                'debug': LaunchConfiguration('debug'),
                'gui': LaunchConfiguration('gui'),
                'paused': 'false',
                'use_sim_time': 'true',
                'headless': LaunchConfiguration('headless'),
                'world_name': os.path.join(map2gazebo_share, 'worlds', 'map.sdf'),
            }.items()
        ),
    ])
