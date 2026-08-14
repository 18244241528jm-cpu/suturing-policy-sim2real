from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package="tf2_ros",executable="static_transform_publisher",name="mock_handeye_tf",
             arguments=["--x","0","--y","0","--z","0","--qx","0","--qy","0","--qz","0","--qw","1",
                        "--frame-id","PSM1_psm_base_link","--child-frame-id","ecm_left_optical_frame"]),
        Node(package="suturing_runtime",executable="mock_topic_source",output="screen"),
        Node(package="suturing_runtime",executable="approach_goal_builder",output="screen"),
        Node(package="suturing_runtime",executable="pipeline_supervisor",output="screen"),
        Node(package="suturing_runtime",executable="guarded_pose_executor",output="screen",parameters=[{"enable_output":False}]),
    ])
