import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("humbot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("humbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "false",
        }.items()
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("humbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
        "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("humbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam) #used when use_slam == FALSE
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("humbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam) #when use slam arg is TRUE
    )

    navigation= IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("humbot_navigation"),
            "launch",
            "navigation.launch.py"
        )
    )

    safety_stop = Node(
        package="humbot_utils",
        executable="safety_stop",
        output="screen"
    )

    rviz_localization = Node(  #used when use_slam == FALSE
        package="rviz2",
        executable="rviz2",
        name="rviz_localization", 
        arguments=["-d", os.path.join(
            get_package_share_directory("humbot_localization"),
            "rviz",
            "global_localization.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(     #used when use_slam == TRUE
        package="rviz2",
        executable="rviz2",
        name="rviz_slam", 
        arguments=["-d", os.path.join(
                get_package_share_directory("humbot_mapping"),
                "rviz",
                "slam.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)   
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        safety_stop,
        rviz_localization,
        rviz_slam
    ])
