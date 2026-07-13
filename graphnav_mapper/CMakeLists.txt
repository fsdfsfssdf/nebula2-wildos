cmake_minimum_required(VERSION 3.8)
project(graphnav_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pose_graph_msgs REQUIRED)
find_package(graphnav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nanoflann REQUIRED)


add_library(graphnav_mapper_component SHARED
  src/mapper.cpp
  src/mapper_node.cpp
  src/navigation_graph.cpp
)
target_include_directories(graphnav_mapper_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(graphnav_mapper_component
  rclcpp
  rclcpp_components
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pose_graph_msgs
  graphnav_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
)
target_link_libraries(graphnav_mapper_component
  ${OpenCV_LIBRARIES}
  nanoflann::nanoflann
)
rclcpp_components_register_node(graphnav_mapper_component PLUGIN "graphnav_mapper::MapperNode" EXECUTABLE mapper_node)


install(TARGETS graphnav_mapper_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dummy_pose_graph_publisher.py
  DESTINATION lib/${PROJECT_NAME}
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
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_mapper test/test_mapper.cpp)
  target_link_libraries(${PROJECT_NAME}_test_mapper graphnav_mapper_component)
  ament_target_dependencies(${PROJECT_NAME}_test_mapper
    rclcpp
    grid_map_msgs
    pose_graph_msgs
    graphnav_msgs
    nav_msgs
  )

endif()

ament_package()
