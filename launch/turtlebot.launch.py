from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    webots_ros2_turtlebot_dir = get_package_share_directory('webots_ros2_turtlebot')
    return LaunchDescription([
        Node(
            name='scan_tf_republisher',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.032', '0', '0.172', '0', '0', '0', 'base_link', 'LDS-01'],
        ),

        Node(
            name='imu_tf_republisher',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.032', '0', '0.068', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/main.launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([webots_ros2_turtlebot_dir, '/launch/robot_launch.py']),
        ),

    ])