import os
from pathlib import Path

from launch import LaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    wildos_root = Path(
        os.environ.get(
            "WILDOS_ROOT",
            str(Path.home() / "cfy" / "nebula2-wildos"),
        )
    )
    venv_python = wildos_root / ".venv_ros" / "bin" / "python3"
    python_prefix = f"{venv_python} " if venv_python.exists() else None
    use_sim_time = LaunchConfiguration('use_sim_time')
    ns = LaunchConfiguration('ns')
    do_object_search = LaunchConfiguration('do_object_search')
    log_level = LaunchConfiguration('log_level')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'ns',
            default_value='',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'do_object_search',
            default_value='false',
            description='Enable object search and launch obj_mask_triangulation node'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='INFO',
            description='Logging level (DEBUG, INFO, WARN, ERROR, FATAL)'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='wildos_nav_conf.yaml',
            description='WildOS YAML configuration file'
        ),

        # Always launch wildos
        Node(
            package='visual_navigation',
            executable='wildos',
            prefix=python_prefix,
            output='screen',
            arguments=[
                '--config', config,
                '--do_object_search', do_object_search,
                '--ros-args', '--log-level', log_level
            ],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/tf', PathJoinSubstitution([TextSubstitution(text='/'), ns, TextSubstitution(text='tf')])),
                ('/tf_static', PathJoinSubstitution([TextSubstitution(text='/'), ns, TextSubstitution(text='tf_static')])),
            ]
        ),

        # Conditionally launch obj_mask_triangulation
        Node(
            package='visual_navigation',
            executable='obj_mask_triangulation',
            prefix=python_prefix,
            output='screen',
            arguments=[
                '--ros-args', '--log-level', log_level
            ],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/tf', PathJoinSubstitution([TextSubstitution(text='/'), ns, TextSubstitution(text='tf')])),
                ('/tf_static', PathJoinSubstitution([TextSubstitution(text='/'), ns, TextSubstitution(text='tf_static')])),
            ],
            condition=IfCondition(do_object_search)
        ),
    ])
