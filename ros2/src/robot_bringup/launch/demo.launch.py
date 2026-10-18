"""Full demo: Gazebo sim + robot_control + robot_agent (+ optional RViz)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_agent = get_package_share_directory("robot_agent")

    default_world = os.path.join(pkg_bringup, "worlds", "simple_world.world")
    default_llm_config = os.path.join(pkg_agent, "config", "llm.yaml")
    default_agent_config = os.path.join(pkg_agent, "config", "agent.yaml")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "verbose": LaunchConfiguration("verbose"),
        }.items(),
    )

    robot_control = Node(
        package="robot_control",
        executable="robot_control_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    robot_agent = Node(
        package="robot_agent",
        executable="robot_agent_node",
        output="screen",
        parameters=[
            {
                "llm_config": LaunchConfiguration("llm_config"),
                "llm_provider": LaunchConfiguration("llm_provider"),
                "mode": LaunchConfiguration("agent_mode"),
                "agent_config": LaunchConfiguration("agent_config"),
                "tool_scan_topic": LaunchConfiguration("tool_scan_topic"),
                "cancel_service": LaunchConfiguration("cancel_service"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="false"),
            DeclareLaunchArgument("llm_config", default_value=default_llm_config),
            DeclareLaunchArgument("llm_provider", default_value=""),
            DeclareLaunchArgument("agent_mode", default_value="simple"),
            DeclareLaunchArgument("agent_config", default_value=default_agent_config),
            DeclareLaunchArgument("tool_scan_topic", default_value="/scan"),
            DeclareLaunchArgument("cancel_service", default_value="/task_cancel"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            sim,
            robot_control,
            robot_agent,
        ]
    )
