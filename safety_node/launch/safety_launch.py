from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # Create a safety node
    safety = Node(
        package='safety_node',
        executable='scripts/safety_node.py',
        name='safety')

    # Add Actions
    ld.add_action(safety)

    return ld
