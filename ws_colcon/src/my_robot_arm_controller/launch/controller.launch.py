from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_arm_controller',
            executable='controller_node',
            name='robot_arm_controller',
            output='screen',
            emulate_tty=True,
        )
    ])
