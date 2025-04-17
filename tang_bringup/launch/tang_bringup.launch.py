from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node

def generate_launch_description():
    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("urg_node2").find("urg_node2"),
                "launch",
                "urg_node2.launch.py"
            )
        )
    )
    
    return LaunchDescription([
        urg_launch,
        ExecuteProcess(
            cmd=["ros2", "run", "tang_control", "tang_control"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "icart_mini_leg_tracker", "leg_cluster_tracking_node"],
            output="screen",
        ),
    ])