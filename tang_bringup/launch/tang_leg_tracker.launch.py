import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from tang_control.config import Control


def generate_launch_description():
    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("urg_node2"),
                "launch",
                "urg_node2.launch.py",
            ])
        )
    )

    # base_linkはTANGの旋回中心。laserは実機計測で前方0.32m。
    static_baselink_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_link_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen",
    )

    static_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser_tf",
        arguments=[
            "0.32", "0", "0",
            "0", "0", "3.1415",
            "base_link", "laser",
        ],
        output="screen",
    )

    leg_tracker_node = Node(
        package="icart_mini_leg_tracker",
        executable="leg_cluster_tracking_node",
        name="leg_cluster_tracking_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("icart_mini_leg_tracker"),
                "config",
                "leg_cluster_tracking_params.yaml",
            ]),
            {
                # TANGでのみ、旋回中心基準の距離連動追従を有効にする。
                "distance_aware_control": True,
                "control_reference_offset_x_m": 0.32,
                "near_enter_distance_m": 0.55,
                "near_exit_distance_m": 0.65,
                "align_start_angle_rad": math.radians(5.0),
                "align_stop_angle_rad": math.radians(3.0),
                "extreme_angle_rad": Control.follow_extreme_angle_rad,
                # configのHIGH用上限まで指令を生成し、TANG制御側で
                # LOW/HIGHそれぞれの最終上限を適用する。
                "follow_max_linear_mps": Control.follow_high_max_v_mps,
                "follow_min_linear_mps": 0.05,
                "follow_max_angular_radps": Control.follow_normal_max_w_radps,
                "follow_extreme_angular_radps": Control.follow_extreme_max_w_radps,
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        urg_launch,
        static_baselink_tf,
        static_laser_tf,
        leg_tracker_node,
    ])
