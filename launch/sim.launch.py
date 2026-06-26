import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("auv_sim_bench")
    config_arg = DeclareLaunchArgument("config", default_value="sim.yaml")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false")

    config = PathJoinSubstitution([share, "config", LaunchConfiguration("config")])
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("auv_sim_bench"), "rviz", "auv_sim.rviz"]
    )

    return LaunchDescription([
        config_arg,
        rviz_arg,
        Node(
            package="auv_sim_bench",
            executable="sim",
            name="auv_sim_bench",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
