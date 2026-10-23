from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ins_benchmark',
            executable='ins_benchmark_node',
            name='ins_benchmark_node',
            parameters=[{'config': 'config/benchmark_config.yaml'}],
        )
    ])
