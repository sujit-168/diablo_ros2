import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    point_to_laser_dir = os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch',)
    livox_dir = os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2',)
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([livox_dir, '/msg_MID360_launch.py']),
        PythonLaunchDescriptionSource([point_to_laser_dir, '/sample_pointcloud_to_laserscan_launch.py']),  
    )
    return LaunchDescription(
        [
            
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
