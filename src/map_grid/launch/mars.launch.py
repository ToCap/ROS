from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_grid',
            executable='map_grid_node',
            name='map_grid_node1',
            output='screen'
        ),
        Node(
            package='pose',
            executable='pose_node',
            name='pose_node1',
            output='screen'
        )
    ])
