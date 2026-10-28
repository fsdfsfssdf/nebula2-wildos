from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Object search in the fork scene, where the target is hidden at spawn."""

    package_share = Path(get_package_share_directory("wildos_simulation"))
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

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
            # The red house is deliberately not visible from the spawn point,
            # so neither the detector nor the frontier selector can produce a
            # first goal on its own. The bootstrap node drives short hops until
            # the detector reports the object.
            "enable_object_search_bootstrap": "true",
            "enable_frontier_exploration": "true",
            "enable_metrics": "true",
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
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
                default_value="fork_01.sdf",
                description="Fork scene with the red house hidden behind the left branch.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            # The rover's lowest point is 0.32 m below base_link and the ground
            # plane sits at z=-0.03, so 0.40 leaves a 5 cm drop instead of the
            # 33 cm free fall that used to flip the robot on the first frame.
            DeclareLaunchArgument("spawn_z", default_value="0.62"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            simulation,
            navigation,
        ]
    )