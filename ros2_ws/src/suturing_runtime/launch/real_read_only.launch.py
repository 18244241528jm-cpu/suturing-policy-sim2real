import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    config=LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config",default_value=os.path.join(
            get_package_share_directory("suturing_runtime"),"config","jhu_real.yaml")),
        Node(package="suturing_runtime",executable="dvrk_topic_adapter",name="dvrk_topic_adapter",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="initialization_snapshot",name="initialization_snapshot",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="metric_da_depth",name="metric_da_depth",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="operator_mask_publisher",name="operator_mask_publisher",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="perception_input_adapter",name="perception_input_adapter",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="fp_bundle_join",name="fp_bundle_join",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="fp_candidate_adapter",name="fp_candidate_adapter",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="needle_pose_selector",name="needle_pose_selector",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="psm_camera_bridge",name="psm_camera_bridge",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="psm_pose_selector",name="psm_pose_selector",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="approach_goal_builder",name="approach_goal_builder",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="pipeline_supervisor",name="pipeline_supervisor",parameters=[config],output="screen"),
        Node(package="suturing_runtime",executable="guarded_pose_executor",name="guarded_pose_executor",parameters=[config,{"enable_output":False}],output="screen"),
    ])
