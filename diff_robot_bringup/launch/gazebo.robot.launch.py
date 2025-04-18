import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    
    use_slam = LaunchConfiguration("use_slam")
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_robot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_robot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_robot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    
    safety_stop = Node(
        package="diff_robot_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_robot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )
    
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_robot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )
    
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_robot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )


    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        navigation,
        rviz,
    ])