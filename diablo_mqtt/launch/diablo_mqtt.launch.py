#!/usr/bin/env/ python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions

def generate_launch_description():
    """Generate a launch description for 3 single diablo mqtt driver."""

    mqtt_receive_node = actions.Node(
        package='diablo_mqtt',
        executable='mqtt_receive',
        output='screen',
        )
    mqtt_send_node = actions.Node(
        package='diablo_mqtt',
        executable='mqtt_send',
        output='screen',
        )

    return LaunchDescription([mqtt_receive_node, mqtt_send_node])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)