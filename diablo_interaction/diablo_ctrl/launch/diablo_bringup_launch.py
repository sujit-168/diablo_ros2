import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    livox_dir = os.path.join(get_package_share_directory('livox_ros_driver2'),'launch_ROS2')
    
    return LaunchDescription(
        [
        IncludeLaunchDescription(
        	PythonLaunchDescriptionSource([livox_dir,'/msg_MID360_launch.py'])
        	),
            Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
        ),
            Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox/lidar'),
                        ],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 0.1,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 3.1415,  # M_PI/2
                'angle_increment': 0.005,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
            Node(
                package = 'diablo_convert',
                executable = 'msg_convert_node',
                output = 'screen'
            ),
            Node(
                package = 'diablo_odom',
                executable = 'odom_publish_node',
                output = 'screen'
            ),
            Node(
                package = 'diablo_ctrl',
                executable = 'diablo_ctrl_node',
                output = 'screen'
            )

        ]
    )
