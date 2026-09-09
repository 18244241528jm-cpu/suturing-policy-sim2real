"""Guarded real-hardware pipeline with frozen Approach-policy inference."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("suturing_runtime")
    config = LaunchConfiguration("config")
    acknowledgement = LaunchConfiguration("operator_acknowledgement")
    model_path = LaunchConfiguration("model_path")
    model_sha256 = LaunchConfiguration("model_sha256")
    surgicai_rl_path = LaunchConfiguration("surgicai_rl_path")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "jhu_real.yaml")),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("model_sha256", default_value=""),
        DeclareLaunchArgument(
            "surgicai_rl_path",
            default_value=os.path.expanduser("~/suturing-policy-sim2real/src/SurgicAI/RL"),
        ),
        DeclareLaunchArgument("operator_acknowledgement", default_value="NOT_ACKNOWLEDGED"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "real_guarded.launch.py")),
            launch_arguments={
                "config": config,
                "operator_acknowledgement": acknowledgement,
                "executor_goal_topic": "/suturing/policy/next_goal",
                "require_completion_signal": "true",
                "reset_timeout_on_goal": "true",
            }.items(),
        ),
        Node(
            package="suturing_runtime",
            executable="approach_policy_node",
            name="approach_policy_node",
            parameters=[config, {
                "enabled": True,
                "enable_policy_output": True,
                "auto_step": True,
                "model_path": model_path,
                "model_sha256": model_sha256,
                "surgicai_rl_path": surgicai_rl_path,
                "operator_acknowledgement": acknowledgement,
            }],
            output="screen",
        ),
    ])
