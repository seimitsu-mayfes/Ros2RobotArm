import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'my_robot_arm_description'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'robot.urdf.xacro'

    urdf = os.path.join(pkg_dir, 'urdf', urdf_file_name)
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str)
            }]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'display.rviz')]]
        )
    ])
