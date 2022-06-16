from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('cnc_interface'),
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='cnc_interface',
            executable='cnc_interface_node',
            name='cnc_interface_node',
            parameters=[params]
        ),
    ])