from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_follower_pkg',
            executable='lane_follower',
            name='lane_follower',
            output='screen'
        )
    ])
