from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_dry_run = LaunchConfiguration("motor_dry_run")

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
        parameters=[{
            "motor_dry_run": ParameterValue(motor_dry_run, value_type=bool),
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "motor_dry_run",
            default_value="false",
            description="Keep RS-485 motor output disabled when true.",
        ),
        leg_tracker_launch,
        tang_control_node,
    ])
