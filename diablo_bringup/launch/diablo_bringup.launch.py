import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    livox_dir = os.path.join(get_package_share_directory('livox_ros_driver2'),'launch_ROS2')
    realsense2_cam_dir = os.path.join(get_package_share_directory('realsense2_camera'),'launch')
    nmea_dir = os.path.join(get_package_share_directory('nmea_navsat_driver'), 'launch')
    robot_ekf_dir = os.path.join(get_package_share_directory('robot_localization'))
    ekf_param_dir = os.path.join(get_package_share_directory('diablo_bringup'),'/config')
    mqtt_dir = os.path.join(get_package_share_directory('diablo_mqtt'),'launch')
    diablo_ctrl_dir = os.path.join(get_package_share_directory('diablo_bringup'),'launch')
    diablo_tf_dir = os.path.join(get_package_share_directory('diablo_description'),'launch')

    ekf_config_dir = LaunchConfiguration(
        'params_file',
        default = [ekf_param_dir,'/diablo_ekf.yaml']
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([livox_dir,'/msg_MID360_launch.py'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense2_cam_dir,'/rs_launch.py'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nmea_dir,'/nmea_serial_driver.launch.py'])
            ),
                
            # mqtt
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([mqtt_dir,'/diablo_mqtt.launch.py'])
            ),

            # diablo_ctrl
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([diablo_ctrl_dir,'/diablo_ctrl.launch.py'])
            ),

            # diablo_tf
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([diablo_tf_dir,'/diablo_tf.launch.py'])
            ),

            # robot_localization
            IncludeLaunchDescription(
            	PythonLaunchDescriptionSource([robot_ekf_dir,'/launch/ekf.launch.py']),
            	launch_arguments = {
                    'params_file':ekf_config_dir
                }.items(),
            ),
                
            Node(
                package='pointcloud_to_laserscan', 
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', '/livox/lidar'),
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
        ]
    )