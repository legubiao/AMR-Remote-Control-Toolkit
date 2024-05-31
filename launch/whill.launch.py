from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

import os

home_dir = os.path.expanduser("~")
foldername = os.path.join(home_dir, "maps")
if not os.path.exists(foldername):
    os.makedirs(foldername)


def generate_launch_description():
    whill_dir = get_package_share_directory('ros_whill')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/main.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([whill_dir, '/launch/basic.launch.py']),
        ),

        Node(
            package='amr_rctk',
            executable='mapping_node.py',
            name='mapping_node',
            output='screen',
            parameters=[
                {'foldername': foldername},
                {'navigation_command': 'amr_rctk navigation2.launch.py use_sim_time:=false'},
                {'start_mapping_command': 'ros2 launch ros_whill cartographer.launch.py include_basic_launch:=false'},
                {'save_map_command': 'ros2 run nav2_map_server map_saver_cli -f'}
            ]
        )
    ])
