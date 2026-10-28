cmake_minimum_required(VERSION 3.8)
project(wildos_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md TERRAIN.md METRICS.md SCENE_TOPOLOGY_PLAN.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_tf.py
  scripts/elevation_relay.py
  scripts/object_search_bootstrap.py
  scripts/pose_to_nav2.py
  scripts/semantic_goal_navigator.py
  scripts/object_goal_adapter.py
  scripts/task_monitor.py
  scripts/ground_filter.py
  scripts/trav_obstacle_relay.py
  scripts/select_frontier_goal.py
  scripts/run_terrain_level.sh
  scripts/odom_recorder.py
  scripts/wildos_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
