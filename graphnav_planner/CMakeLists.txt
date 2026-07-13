cmake_minimum_required(VERSION 3.8)
project(graphnav_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(graphnav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graaf_vendor REQUIRED)


add_library(graphnav_planner_component SHARED
  src/planner.cpp
  src/planner_node.cpp
)
target_include_directories(graphnav_planner_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(graphnav_planner_component
  rclcpp
  rclcpp_components
  graphnav_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  grid_map_msgs
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  graaf_vendor
)
target_link_libraries(graphnav_planner_component)
rclcpp_components_register_node(graphnav_planner_component PLUGIN "graphnav_planner::PlannerNode" EXECUTABLE planner_node)


add_library(path_follower_component SHARED src/path_follower_node.cpp)
target_include_directories(path_follower_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(path_follower_component
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(path_follower_component)
rclcpp_components_register_node(path_follower_component PLUGIN "graphnav_planner::PathFollowerNode" EXECUTABLE path_follower_node)



# Install targets
install(TARGETS
  graphnav_planner_component
  planner_node
  path_follower_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
