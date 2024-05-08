from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reisen_ros',
            executable='control',
        ),
        Node(
            package='reisen_ros',
            executable='serial',
        ),
        Node(
            package='reisen_ros',
            executable='odom',
        ),
    ])
