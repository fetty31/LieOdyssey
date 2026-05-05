from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz_config = LaunchConfiguration('rviz')
    param_config = LaunchConfiguration('config')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description = 'Whether to run an rviz instance'
    )

    param_config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
                FindPackageShare('lio_ros'),
                'config',
                'params.yaml'
            ]),
        description = 'Path to yaml config'
    )

    lio_node = Node(
        package='lio_ros',
        namespace='',
        executable='lio_ros_node',
        name='lio_ros_node',
        output='screen',
        parameters=[param_config]
    )

    rviz_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                rviz_config
            ])
        ),
        cmd=[[
            'ros2 run rviz2 rviz2 -d ',
             PathJoinSubstitution([
                FindPackageShare('lio_ros'),
                'config',
                'rviz',
                'limo.rviz'
            ])
        ]],
        shell=True
    )

    
    return LaunchDescription([
        rviz_config_arg,
        param_config_arg,
        lio_node,
        rviz_conditioned
    ])