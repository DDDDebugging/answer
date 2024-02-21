from launch import LaunchDescription
import launch_ros.actions import Node

def generate_launch_description():
    answer_node = Node(
    	package="answer",
    	executable="answer_node"
    	)
    homework_node = Node(
    	package="homework",
    	executable="homework_node"
    	)
    launch_description = LaunchDescription([answer_node,homework_node])
    return launch_description
