from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    home = os.path.expanduser("~")

    urdf_path = os.path.join(
        home, "ros2_ws", "src", "mren203_new", "scripts", "my_robot.urdf"
    )
    slam_params_path = os.path.join(
        home, "ros2_ws", "src", "mren203_new", "scripts", "mapper_params_online_async.yaml"
    )
    nav2_params_path = os.path.join(
        home, "ros2_ws", "src", "mren203_new", "scripts", "nav2_params.yaml"
    )

    with open(urdf_path, "r") as f:
        robot_description_content = f.read()

    sllidar_launch_path = os.path.join(
        get_package_share_directory("sllidar_ros2"),
        "launch",
        "view_sllidar_a1_launch.py",
    )

    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )

    nav2_launch_path = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )

    foxglove_launch_path = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )

    use_lidar = LaunchConfiguration("use_lidar")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_foxglove = LaunchConfiguration("use_foxglove")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "127"),

        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            description="Launch the SLLIDAR driver",
        ),
        DeclareLaunchArgument(
            "use_slam",
            default_value="true",
            description="Launch slam_toolbox",
        ),
        DeclareLaunchArgument(
            "use_nav2",
            default_value="true",
            description="Launch Nav2",
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="true",
            description="Launch foxglove_bridge",
        ),

        Node(
            package="mren203_new",
            executable="combined_serial_bridge",
            name="combined_serial_bridge",
            output="screen",
        ),

        Node(
            package="mren203_new",
            executable="odom_publisher",
            name="odom_publisher",
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_content
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
            condition=IfCondition(use_lidar),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            condition=IfCondition(use_slam),
            launch_arguments={
                "slam_params_file": slam_params_path,
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(foxglove_launch_path),
            condition=IfCondition(use_foxglove),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": nav2_params_path,
            }.items(),
        ),
    ])