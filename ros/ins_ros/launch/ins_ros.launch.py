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
                FindPackageShare('ins_ros'),
                'config',
                'kitti.yaml'
            ]),
        description = 'Path to yaml config'
    )

    node = Node(
        package='ins_ros',
        namespace='',
        executable='ins_ros_node',
        name='ins_ros_node',
        output='screen',
        parameters=[param_config],
        arguments=[
            '--ros-args',
            '--log-level',
            'ins_ros_node:=debug'
        ]
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
                FindPackageShare('ins_ros'),
                'config',
                'rviz',
                'template.rviz'
            ])
        ]],
        shell=True
    )

    
    return LaunchDescription([
        rviz_config_arg,
        param_config_arg,
        node,
        rviz_conditioned
    ])