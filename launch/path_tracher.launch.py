from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='amr_rctk',
            parameters=[{'use_sim_time': use_sim_time}],
            executable='path_publisher',
            name='path_publisher',
            namespace='amr_rctk'
        ),
        Node(
            package='amr_rctk',
            parameters=[{'use_sim_time': use_sim_time}],
            executable='path_tracker_pid',
            name='path_tracker_pid',
            namespace='amr_rctk'
        )
    ])
