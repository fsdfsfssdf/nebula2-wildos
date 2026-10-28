from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory("wildos_simulation"))
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    world_name = LaunchConfiguration("world_name")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "minimal_sim.launch.py")
        ),
        launch_arguments={
            "rviz": rviz,
            "headless": headless,
            "enable_mapping": "true",
            "enable_graph": "true",
            "enable_visual_scoring": "true",
            "do_object_search": "true",
            "enable_planner": "true",
            "planner_graph_topic": "/spot1/scored_nav_graph",
            "planner_goal_topic": "/spot1/task_goal_pose",
            "enable_object_goal_adapter": "true",
            "enable_object_search_bootstrap": "false",
            "enable_frontier_exploration": "true",
            "world_name": world_name,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "wildos_nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "enable_pose_bridge": "true",
            "enable_task_goal": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "world_name",
                default_value="minimal_field_hidden.sdf",
                description=(
                    "World file. Use terrain_01_gentle.sdf, "
                    "terrain_02_rough.sdf or terrain_03_challenge.sdf "
                    "to increase terrain difficulty."
                ),
            ),
            simulation,
            navigation,
        ]
    )
