from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    # 実機で使用していた固定TF。laserはbase_linkから180度回転させる。
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
            "0", "0", "0",
            "0", "0", "3.1415",
            "base_link", "laser",
        ],
        output="screen",
    )

    leg_tracker_node = Node(
        package="icart_mini_leg_tracker",
        executable="leg_cluster_tracking_node",
        name="leg_cluster_tracking_node",
        output="screen",
    )

    return LaunchDescription([
        urg_launch,
        static_baselink_tf,
        static_laser_tf,
        leg_tracker_node,
    ])
