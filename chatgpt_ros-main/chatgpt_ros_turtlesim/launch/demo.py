from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chatgpt_ros',
            namespace='/',
            executable='chatgpt_action_server',
            name='chatgpt_action_server'
        ),
        Node(
            package='turtlesim',
            namespace='/',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='chatgpt_ros_turtlesim',
            namespace='/',
            executable='chat_turtle',
            name='chat_turtle',
            # 使用 gnome-terminal 命令在新的终端中启动节点
            prefix="gnome-terminal --"
        )
    ])