from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    stanleyControll = os.path.join(get_package_share_directory('stanley_control_niagara'),'config','stanleyControl.yaml')

    return LaunchDescription([
        Node(
            package='stanley_control_niagara',
            executable='stanley_control_niagara_node',
            name='stanley_control_niagara_node',
            output='screen',
            parameters=[stanleyControll]
        )
    ])