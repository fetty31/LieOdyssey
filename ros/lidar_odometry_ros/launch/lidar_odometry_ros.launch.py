from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz_config = LaunchConfiguration('rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description = 'Whether to run an rviz instance'
    )

    lio_node = Node(
        package='lidar_odometry_ros',
        namespace='',
        executable='lidar_odometry_ros_node',
        name='lidar_odometry_ros',
        output='screen',
        parameters=[PathJoinSubstitution([
                FindPackageShare('lidar_odometry_ros'),
                'config',
                'params.yaml'
            ])]
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
                FindPackageShare('lidar_odometry_ros'),
                'config',
                'rviz',
                'limo.rviz'
            ])
        ]],
        shell=True
    )

    
    return LaunchDescription([
        rviz_config_arg,
        lio_node,
        rviz_conditioned
    ])