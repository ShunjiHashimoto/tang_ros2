from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # URG、TF、脚追従ノードは追従用launchにまとめる。
    leg_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tang_bringup"),
                "launch",
                "tang_leg_tracker.launch.py",
            ])
        )
    )

    # GPIO、SPIジョイスティック、モード切替を管理する。
    tang_control_node = Node(
        package="tang_control",
        executable="tang_control",
        name="tang_control",
        output="screen",
    )

    return LaunchDescription([
        leg_tracker_launch,
        tang_control_node,
    ])
