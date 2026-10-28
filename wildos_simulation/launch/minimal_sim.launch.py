from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("wildos_simulation"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    elevation_share = Path(get_package_share_directory("elevation_mapping_cupy"))
    visual_navigation_share = Path(
        get_package_share_directory("visual_navigation")
    )

    world_name = LaunchConfiguration("world_name")
    world = PathJoinSubstitution([str(package_share / "worlds"), world_name])
    robot_xacro = package_share / "urdf" / "d1h" / "d1h_navigation.urdf.xacro"
    robot_description = xacro.process_file(str(robot_xacro)).toxml()

    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_graph = LaunchConfiguration("enable_graph")
    enable_planner = LaunchConfiguration("enable_planner")
    planner_graph_topic = LaunchConfiguration("planner_graph_topic")
    planner_goal_topic = LaunchConfiguration("planner_goal_topic")
    enable_object_goal_adapter = LaunchConfiguration(
        "enable_object_goal_adapter"
    )
    enable_object_search_bootstrap = LaunchConfiguration(
        "enable_object_search_bootstrap"
    )
    enable_frontier_exploration = LaunchConfiguration(
        "enable_frontier_exploration"
    )
    enable_visual_scoring = LaunchConfiguration("enable_visual_scoring")
    visual_config = LaunchConfiguration("visual_config")
    do_object_search = LaunchConfiguration("do_object_search")
    enable_metrics = LaunchConfiguration("enable_metrics")
    metrics_output_dir = LaunchConfiguration("metrics_output_dir")
    headless = LaunchConfiguration("headless")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PythonExpression(
                    ["'-s ' if '", headless, "' == 'true' else ''"]
                ),
                world,
            ]
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description,
            "-name",
            "wildos_rover",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/wildos_rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/wildos_rover/odometry_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/spot1/realsense/front/color/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/spot1/realsense/front/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/spot1/realsense/left/color/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/spot1/realsense/left/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/spot1/realsense/right/color/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/spot1/realsense/right/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/spot1/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/spot1/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/spot1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/wildos_minimal_field/model/wildos_rover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            ("/model/wildos_rover/cmd_vel", "/spot1/cmd_vel"),
            ("/model/wildos_rover/odometry_gt", "/spot1/odom"),
            ("/spot1/realsense/front/color/camera", "/spot1/realsense/front/color/image_raw"),
            ("/spot1/realsense/left/color/camera", "/spot1/realsense/left/color/image_raw"),
            ("/spot1/realsense/right/color/camera", "/spot1/realsense/right/color/image_raw"),
            ("/spot1/lidar", "/spot1/scan"),
            ("/spot1/lidar/points", "/spot1/points"),
            ("/world/wildos_minimal_field/model/wildos_rover/joint_state", "/joint_states"),
        ],
    )

    odom_tf = Node(
        package="wildos_simulation",
        executable="odom_tf.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "odom_topic": "/spot1/odom",
                "parent_frame": "odom",
                "child_frame": "base_link",
                "robot_description": robot_description,
            }
        ],
    )

    elevation_mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            str(elevation_share / "config" / "core" / "core_param.yaml"),
            str(package_share / "config" / "elevation_mapping.yaml"),
            {"use_sim_time": True},
        ],
        condition=IfCondition(enable_mapping),
    )

    elevation_relay = Node(
        package="wildos_simulation",
        executable="elevation_relay.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                # The paper uses 0.7 on field data. The D1H simulation's
                # synthetic LiDAR is sparser, so use a calibrated threshold
                # until a simulation-specific traversability model is trained.
                "traversability_threshold": 0.4,
            }
        ],
        condition=IfCondition(enable_mapping),
    )

    dummy_pose_graph = Node(
        package="graphnav_mapper",
        executable="dummy_pose_graph_publisher.py",
        name="dummy_pose_graph_publisher",
        namespace="spot1",
        output="screen",
        parameters=[{"use_sim_time": True, "frame": "odom"}],
        condition=IfCondition(enable_graph),
    )

    graphnav_mapper = Node(
        package="graphnav_mapper",
        executable="mapper_node",
        name="graphnav_mapper",
        namespace="spot1",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                # Permit the final 0.8 m of a frontier association ray to
                # touch the unknown boundary. The library default remains 0.
                "frontier_end_margin": 0.8,
            }
        ],
        remappings=[
            ("odometry", "odom"),
            # The mapper expects obstacle cost 0=safe and 1=blocked. The relay
            # publishes this alongside the visualization/Nav2 occupancy map.
            ("occupancy", "unused_local_map"),
            ("traversability", "navigation_cost_map"),
        ],
        condition=IfCondition(enable_graph),
    )

    graphnav_planner = Node(
        package="graphnav_planner",
        executable="planner_node",
        name="planner_node",
        namespace="spot1",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "goal_radius": 1.5,
                "path_smoothness_period": 0.0,
            }
        ],
        remappings=[
            (
                "/spot1/planner_node/nav_graph",
                planner_graph_topic,
            ),
            (
                "/spot1/planner_node/goal_pose",
                planner_goal_topic,
            ),
            (
                "/spot1/planner_node/odom",
                "/spot1/odom",
            ),
            (
                "/spot1/planner_node/path",
                "/spot1/graphnav_path",
            ),
        ],
        condition=IfCondition(enable_planner),
    )

    graphnav_path_follower = Node(
        package="graphnav_planner",
        executable="path_follower_node",
        name="path_follower_node",
        namespace="spot1",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "wp_lookahead_dist": 2.0,
                "path_timeout": 5.0,
            }
        ],
        remappings=[
            (
                "/spot1/path_follower_node/path",
                "/spot1/graphnav_path",
            ),
            (
                "/spot1/path_follower_node/odom",
                "/spot1/odom",
            ),
            (
                "/spot1/path_follower_node/goal_pose",
                "/spot1/goal_pose",
            ),
        ],
        condition=IfCondition(enable_planner),
    )

    object_goal_adapter = Node(
        package="wildos_simulation",
        executable="object_goal_adapter.py",
        name="object_goal_adapter",
        namespace="spot1",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_object_goal_adapter),
    )

    object_search_bootstrap = Node(
        package="wildos_simulation",
        executable="object_search_bootstrap.py",
        name="object_search_bootstrap",
        namespace="spot1",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_object_search_bootstrap),
    )

    frontier_exploration = Node(
        package="wildos_simulation",
        executable="select_frontier_goal.py",
        name="frontier_exploration_selector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "use_task_goal": False,
                "output_topic": "/spot1/frontier_exploration_goal",
            }
        ],
        condition=IfCondition(enable_frontier_exploration),
    )

    visual_scoring = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                visual_navigation_share
                / "launch"
                / "wildos_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "config": visual_config,
            "do_object_search": do_object_search,
            "log_level": "INFO",
        }.items(),
        condition=IfCondition(enable_visual_scoring),
    )

    # Trajectory logging: the raw evidence every run needs for scoring.
    # One CSV row per odometry sample (~30 Hz, 11 columns); wildos_metrics.py
    # turns it into path length, detour ratio, stall events, attitude and
    # clearance, so a run can be re-scored without simulating again.
    odom_recorder = Node(
        package="wildos_simulation",
        executable="odom_recorder.py",
        name="odom_recorder",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_dir": metrics_output_dir,
                "world": world_name,
                "odom_topic": "/spot1/odom",
                "cmd_topic": "/spot1/cmd_vel",
                "imu_topic": "/spot1/imu",
                "scan_topic": "/spot1/scan",
            }
        ],
        condition=IfCondition(enable_metrics),
    )

    # Traversability comes from the elevation map, exactly the layer that
    # graphnav_mapper uses for its nodes and edges. Nav2 is our robot-side
    # navigator, so it has to see the same "this is not drivable" cells.
    trav_relay = Node(
        package="wildos_simulation",
        executable="trav_obstacle_relay.py",
        name="trav_obstacle_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/spot1/local_map",
                "output_topic": "/spot1/trav_obstacles",
            }
        ],
        condition=IfCondition(enable_mapping),
    )

    # The costmaps need obstacles in the odom frame, whose z origin sits on the
    # robot base rather than on the ground (terrain floats between -0.61 and
    # -0.40 m). A fixed height gate therefore either drops low rocks or marks
    # the road itself, so the ground is removed here instead.
    ground_filter = Node(
        package="wildos_simulation",
        executable="ground_filter.py",
        name="ground_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/spot1/points",
                "output_topic": "/spot1/points_obstacles",
                "min_height_above_ground": 0.12,
            }
        ],
    )

    # Prints exactly one terminal line ("TASK COMPLETE" / "STALLED" /
    # "TIMEOUT") so a run has an unambiguous ending.
    task_monitor = Node(
        package="wildos_simulation",
        executable="task_monitor.py",
        name="task_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "odom_topic": "/spot1/odom",
                "reached_topic": "/spot1/goal_reached",
                "target_topic": "/spot1/imgnav_waypoint",
            }
        ],
        condition=IfCondition(enable_metrics),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [str(package_share / "rviz"), rviz_config]
            ),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="wildos_visualization.rviz",
            ),
            DeclareLaunchArgument("enable_mapping", default_value="false"),
            DeclareLaunchArgument("enable_graph", default_value="false"),
            DeclareLaunchArgument("enable_planner", default_value="false"),
            DeclareLaunchArgument(
                "planner_graph_topic",
                default_value="/spot1/nav_graph",
            ),
            DeclareLaunchArgument(
                "planner_goal_topic",
                default_value="/spot1/graph_goal_pose",
            ),
            DeclareLaunchArgument(
                "enable_object_goal_adapter",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_object_search_bootstrap",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_frontier_exploration",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_visual_scoring",
                default_value="false",
            ),
            DeclareLaunchArgument("enable_metrics", default_value="true"),
            DeclareLaunchArgument(
                "metrics_output_dir",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable(
                            "WILDOS_ROOT", default_value="."
                        ),
                        "outputs",
                        "metrics",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "visual_config",
                default_value="wildos_nav_d1h.yaml",
            ),
            DeclareLaunchArgument("do_object_search", default_value="false"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "world_name",
                default_value="minimal_field.sdf",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.62"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            gz_sim,
            spawn_robot,
            bridge,
            odom_tf,
            elevation_mapping,
            elevation_relay,
            dummy_pose_graph,
            graphnav_mapper,
            graphnav_planner,
            graphnav_path_follower,
            object_goal_adapter,
            object_search_bootstrap,
            frontier_exploration,
            visual_scoring,
            odom_recorder,
            task_monitor,
            ground_filter,
            trav_relay,
            rviz_node,
        ]
    )
