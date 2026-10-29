from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('uav_sim_visualization')) / 'config/sensors.rviz'
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('px4_namespace', default_value='/px4_1'),
        DeclareLaunchArgument('rviz_config', default_value=str(config)),
        Node(package='uav_sim_visualization', executable='sensor_frames', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'px4_namespace': LaunchConfiguration('px4_namespace')}]),
        Node(package='rviz2', executable='rviz2', name='sensor_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
