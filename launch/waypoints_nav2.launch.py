
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    limo_bringup_dir = get_package_share_directory('limo_bringup')
    nav2_launch_file_dir = os.path.join(limo_bringup_dir, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/limo_nav2.launch.py']),
        ),

        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                Node(
                    package='limo_bringup',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                ),
            ]
        ),
    ])