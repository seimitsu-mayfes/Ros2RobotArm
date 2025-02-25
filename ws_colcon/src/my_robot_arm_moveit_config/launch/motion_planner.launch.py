import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージのパスを取得
    description_package = get_package_share_directory("my_robot_arm_description")
    moveit_config_package = get_package_share_directory("my_robot_arm_moveit_config")

    # URDFとRVizの設定ファイルのパス
    urdf_path = os.path.join(description_package, "urdf", "robot.urdf.xacro")
    rviz_config = os.path.join(moveit_config_package, "config", "moveit.rviz")
    srdf_path = os.path.join(moveit_config_package, "config", "my_robot_arm.srdf")

    # MoveIt2の設定を読み込み
    moveit_config = (MoveItConfigsBuilder("my_robot_arm", package_name="my_robot_arm_moveit_config")
                    .robot_description(file_path=urdf_path)
                    .robot_description_semantic(srdf_path)
                    .trajectory_execution(os.path.join(moveit_config_package, "config", "controllers.yaml"))
                    .joint_limits(os.path.join(moveit_config_package, "config", "joint_limits.yaml"))
                    .robot_description_kinematics(os.path.join(moveit_config_package, "config", "kinematics.yaml"))
                    .pilz_cartesian_limits(os.path.join(moveit_config_package, "config", "pilz_cartesian_limits.yaml"))
                    .to_moveit_configs())

    # Joint State Publisherの設定
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # Robot State Publisherの設定
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Move Group Nodeの設定
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"trajectory_execution.allowed_goal_duration_margin": 0.5},
            {"trajectory_execution.allowed_start_tolerance": 0.01},
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager": {
                    "controller_names": ["arm_controller"],
                    "arm_controller": {
                        "type": "FollowJointTrajectory",
                        "action_ns": "follow_joint_trajectory",
                        "default": True,
                        "joints": ["joint1", "joint2", "joint3", "joint4", "joint5"]
                    }
                }
            }
        ],
    )

    # RVizの設定
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Motion Planner Nodeの設定
    motion_planner_node = Node(
        package="my_robot_arm_moveit_config",
        executable="motion_planner_node",
        name="motion_planner_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # コントローラノードの設定
    controller_node = Node(
        package="my_robot_arm_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        motion_planner_node,
        controller_node,
    ])
