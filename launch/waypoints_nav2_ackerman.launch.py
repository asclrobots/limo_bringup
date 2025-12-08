import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of the limo_bringup package
    limo_bringup_dir = get_package_share_directory('limo_bringup')

    # Include the main navigation launch file
    # This starts the Nav2 stack, localization, and planners
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_bringup_dir, 'launch', 'limo_nav2_ackmann.launch.py')
        )
    )

    # Define the node that will run your waypoint follower script
    waypoint_follower_node = Node(
        package='limo_bringup',
        executable='waypoint_follower', # The name from setup.py entry_points
        name='waypoint_follower',
        output='screen'
    )

    # Use a TimerAction to delay the start of the waypoint follower
    # This gives the Nav2 stack some time to initialize properly
    delayed_waypoint_follower = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[waypoint_follower_node]
    )

    return LaunchDescription([
        start_nav2_cmd,
        delayed_waypoint_follower
    ])