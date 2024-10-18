import launch
import launch_ros

def generate_launch_description():
    diablo_ctrl_node = launch_ros.actions.Node(
        name='diablo_ctrl_node',
        package='diablo_ctrl',
        executable='diablo_ctrl_node',
        respawn=True,
        respawn_delay=10.0
    )

    teleop_stand_node = launch_ros.actions.Node(
        name='diablo_stand_ctrl_node',
        package='diablo_teleop',
        executable='teleop_stand_node',
        respawn=True,
        respawn_delay=10.0
    )
    
    return launch.LaunchDescription([diablo_ctrl_node, teleop_stand_node])
    # return launch.LaunchDescription([diablo_ctrl_node])

if __name__ == '__main__':
    launch.launch(generate_launch_description())