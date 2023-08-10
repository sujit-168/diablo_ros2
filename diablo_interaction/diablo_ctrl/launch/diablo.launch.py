import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
