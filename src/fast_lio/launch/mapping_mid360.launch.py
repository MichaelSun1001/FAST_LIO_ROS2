import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    system_library_path = [
        '/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:',
        EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
    ]

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time},
        ],
        additional_env={'LD_LIBRARY_PATH': system_library_path},
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Yaml config directory.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='mid360.yaml',
            description='Yaml config file.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=default_rviz_config_path,
            description='RViz config file path.',
        ),
        fast_lio_node,
        rviz_node,
    ])
