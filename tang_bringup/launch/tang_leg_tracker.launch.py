from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

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
    
    static_baselink_tf = ExecuteProcess(
        cmd=[
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0", "0", "0",  "0", "0", "0",
            "map", "base_link"
        ],
        output="screen"
    )
    
    static_laser_tf = ExecuteProcess(
        cmd=[
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.0", "0", "0.0",         # 位置オフセット (x, y, z)
            "0", "0", "3.1415",     # 姿勢オフセット (roll, pitch, yaw) ← Z軸回り180度回転
            "base_link", "laser"    # 親フレーム base_link, 子フレーム laser
        ],
        output="screen"
    )
    
    return LaunchDescription([
        urg_launch,
        static_baselink_tf,
        static_laser_tf,
        ExecuteProcess(
            cmd=["ros2", "run", "icart_mini_leg_tracker", "leg_cluster_tracking_node"],
            output="screen",
        ),
    ])