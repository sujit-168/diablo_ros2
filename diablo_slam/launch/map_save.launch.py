import os
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # get map name from args
    # map_filename_launch_arg = DeclareLaunchArgument(
    #     "map_filename", default_value=TextSubstitution(text="")
    # )

    # map named is map+current time
    current_time = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())
    map_filename = 'map_' + current_time

    # get directory of diablo_navigation
    map_directory = os.path.join(
        get_package_share_directory('diablo_navigation2'), 'maps')
    path = Path(map_directory)

    # operate path instance ，get the path needed
    map_directory = Path(path.parents[4], "src/diablo_ros2", *path.parts[-2:])

    # set and check save files
    os.makedirs(map_directory, exist_ok=True)

    map_save_config = os.path.join(map_directory, map_filename)

    return LaunchDescription(
        [   
            # map_filename_launch_arg,
            # DeclareLaunchArgument("map_saver_params_file", default_value=map_save_config, description="Map Saver Configuration File"),

            Node(
                package='nav2_map_server',
                executable='map_saver_cli',
                name='map_saver_cli',
                # namespace='your_namespace',
                # arguments=['-r', '__ns:=/your_namespace', '-f', map_filename],
                arguments=['-t', 'map', '-f', map_save_config],
                output='screen'
            )
        ]
    )

