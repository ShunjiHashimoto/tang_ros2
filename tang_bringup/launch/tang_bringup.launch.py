from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # joy_node の起動
    joy_node = ExecuteProcess(
        cmd=["ros2", "run", "joy", "joy_node", 
             "--ros-args", "--param", "device_id:=0"],
        output="screen",
    )

    # teleop_twist_joy の起動
    teleop_twist_joy_node = ExecuteProcess(
        cmd=["ros2", "run", "teleop_twist_joy", "teleop_node", 
             "--ros-args", "--params-file", 
             "/root/icart_ws/src/icart_mini_ros2/icart_mini_ypspur_bridge/config/teleop_twist_joy_f710_params.yaml"],
        output="screen"
    )

    # tang_controlノードの起動
    tang_control_node = Node(
        package='tang_control',
        executable='tang_control',
        name='tang_control',  # typo修正: cotrol → control
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node,
        tang_control_node
    ])
