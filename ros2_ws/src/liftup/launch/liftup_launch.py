import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'liftup_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='liftup',
            executable='liftup_node_entrypoint',
            name='liftup_node',
            output='screen',
            parameters=[config_file]
        )
    ])