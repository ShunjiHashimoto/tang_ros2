from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package='tang_control',
        executable='tang_control',
        name='tang_cotrol',
        output='screen',
    )
    ])
    