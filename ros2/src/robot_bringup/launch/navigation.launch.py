"""Launch SLAM (slam_toolbox online_async) + Nav2 navigation stack.

Assumes the robot is already simulated (see sim.launch.py): TF, /scan and /odom
must already be available.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory("robot_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    mapper_params = os.path.join(pkg_bringup, "config", "mapper_params_online_async.yaml")
    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params.yaml")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": mapper_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([slam, nav2])
