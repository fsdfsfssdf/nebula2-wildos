from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("wildos_simulation"))
    params_file = str(package_share / "config" / "nav2_sim.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    enable_pose_bridge = LaunchConfiguration("enable_pose_bridge")
    enable_task_goal = LaunchConfiguration("enable_task_goal")

    common_parameters = [params_file, {"use_sim_time": use_sim_time}]
    tf_remappings = [("/tf", "/tf"), ("/tf_static", "/tf_static")]

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common_parameters,
        remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
    )
    smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=common_parameters,
        remappings=tf_remappings,
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_parameters,
        remappings=tf_remappings,
    )
    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common_parameters,
        remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common_parameters,
        remappings=tf_remappings,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=common_parameters,
        remappings=tf_remappings
        + [
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "/spot1/cmd_vel"),
        ],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"autostart": autostart},
            {
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "velocity_smoother",
                ]
            },
            {"use_sim_time": use_sim_time},
        ],
    )
    pose_bridge = Node(
        package="wildos_simulation",
        executable="pose_to_nav2.py",
        name="pose_to_nav2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_pose_bridge),
    )
    frontier_selector = Node(
        package="wildos_simulation",
        executable="select_frontier_goal.py",
        name="frontier_goal_selector",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "use_task_goal": True,
                "task_goal_x": 28.0,
                "task_goal_y": 0.0,
            }
        ],
        condition=IfCondition(enable_task_goal),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("enable_pose_bridge", default_value="true"),
            DeclareLaunchArgument("enable_task_goal", default_value="true"),
            controller,
            smoother,
            planner,
            behavior,
            bt_navigator,
            velocity_smoother,
            lifecycle_manager,
            pose_bridge,
            frontier_selector,
        ]
    )
