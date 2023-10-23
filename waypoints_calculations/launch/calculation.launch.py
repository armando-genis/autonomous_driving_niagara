from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    waypointsConfig = os.path.join(get_package_share_directory('waypoints_calculations'),'config','waypointsConfig.yaml')

    return LaunchDescription([
        Node(
            package='waypoints_calculations',
            executable='waypoints_calculations',
            name='waypoints_calculations',
            output='screen',
            parameters=[waypointsConfig]
        )
    ])