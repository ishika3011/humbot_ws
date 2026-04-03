import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_roslaunch import get_package_share_directory  # type: ignore

def generate_launch_description():

    pkg_share = get_package_share_directory('humbot_nmpc')
    
    # Path to your params file
    nmpc_params = os.path.join(pkg_share, 'config', 'nmpc_params.yaml')

    # Controller server node — this is the Nav2 node that loads your plugin
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nmpc_params],
    )

    return LaunchDescription([
        controller_server,
    ])