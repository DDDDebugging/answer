from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='answer',
            executable='answer_node',
            name='answer'
        ),
        Node(
            package='homework',
            executable='homework_node',
            name='homework'
        )
    ])
