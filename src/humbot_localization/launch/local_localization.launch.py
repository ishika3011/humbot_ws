from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    static_transform_publisher = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0.103", 
                     "0", "0", "0", "1", 
                     "base_footprint_ekf", "imu_link_ekf"]
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("humbot_localization"), "config", "ekf.yaml")]
    )

    imu_republisher_cpp = Node(
        package = "humbot_localization",
        executable = "imu_republisher",
        
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization_node,
        imu_republisher_cpp

    ])