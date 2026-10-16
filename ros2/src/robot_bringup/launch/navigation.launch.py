"""Launch mapping or saved-map localization with Nav2.

The robot must already be running (see sim.launch.py), with /scan, /odom and TF.
Use mode:=mapping to build a map, then save it with scripts/wsl_save_map.sh.
Use mode:=localization map:=/absolute/path/to/map.yaml after restarting the sim.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _validate_localization_map(context):
    mode = LaunchConfiguration("mode").perform(context)
    map_path = LaunchConfiguration("map").perform(context)
    if mode not in ("mapping", "localization"):
        raise RuntimeError("mode must be either 'mapping' or 'localization'")
    if mode == "localization" and (not map_path or not os.path.isfile(map_path)):
        raise RuntimeError(
            "localization requires map:=/absolute/path/to/map.yaml; "
            f"file does not exist: {map_path!r}"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory("robot_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    mapper_params = os.path.join(pkg_bringup, "config", "mapper_params_online_async.yaml")
    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params.yaml")
    nav2_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    mapping = PythonExpression(["'", LaunchConfiguration("mode"), "' == 'mapping'"])
    localization = PythonExpression(["'", LaunchConfiguration("mode"), "' == 'localization'"])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": mapper_params,
        }.items(),
        condition=IfCondition(mapping),
    )
    localization_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "slam": "False",
            "map": LaunchConfiguration("map"),
            "use_sim_time": "true",
            "params_file": nav2_params,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
        condition=IfCondition(localization),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument("map", default_value=""),
            OpaqueFunction(function=_validate_localization_map),
            slam,
            localization_nav,
        ]
    )
