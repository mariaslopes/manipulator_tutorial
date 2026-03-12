import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    
    mode = DeclareLaunchArgument(
        "mode",
        default_value="0",
        description="planning mode: 0 for planning only, 1 for planning and execution",
    )

    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    task_launch = Node(
        package="manipulator_tutorial",
        executable="task_node",
        output="screen",
        parameters=[
          moveit_config,
          {"mode": LaunchConfiguration("mode")}
        ],
    )

    object_pos_publisher = Node(
        package="manipulator_tutorial",
        executable="scene_to_tf.py",
        output="screen",
    )

    return LaunchDescription(
        [
            mode,
            task_launch,
            object_pos_publisher,
        ]
    )