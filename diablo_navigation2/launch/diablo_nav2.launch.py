import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_dir = os.path.join(get_package_share_directory('diablo_navigation2'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'),'launch')
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'),'rviz','nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default = [nav2_dir,
            '/maps',
            '/tianbotoffice_602.yaml']
        
    )
    param_dir = LaunchConfiguration(
        'params_file',
        default = [nav2_dir,'/config','/diablo_nav2.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument('map',default_value = map_dir,description = 'Full path to map file to load'),
            DeclareLaunchArgument('params_file',default_value = param_dir,description = 'Full path to param file to load'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir,'/bringup_launch.py']),
                launch_arguments = {
                    'map':map_dir,
                    'use_sim_time':use_sim_time,
                    'params_file':param_dir
                }.items(),
            ),
            
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([nav2_dir,'/launch','/diablo_state_publisher.launch.py'])
            # ),

            Node(
                package = 'rviz2',
                executable = 'rviz2',
                arguments = ['-d',rviz_config_dir],
                output = 'screen'
                )
        ])
