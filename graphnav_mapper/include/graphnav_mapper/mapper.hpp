#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pose_graph_msgs/msg/pose_graph.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include "graphnav_mapper/navigation_graph.hpp"
#include "graphnav_mapper/pose_graph.hpp"
#include <grid_map_core/GridMap.hpp>

namespace graphnav_mapper
{

class Mapper
{
public:
  Mapper(tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger);
  void odometry_update(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void traversability_update(const grid_map_msgs::msg::GridMap::ConstSharedPtr map,
                             grid_map_msgs::msg::GridMap::SharedPtr dbg_map_out = nullptr);
  void pose_graph_update(const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr pose_graph);
  // void image_update(const sensor_msgs::msg::CompressedImage& image);
  visualization_msgs::msg::MarkerArray get_visualization(rclcpp::Time stamp, bool with_id_text = false) const;
  graphnav_msgs::msg::NavigationGraph get_graph_msg(rclcpp::Time stamp) const;

  graphnav::NavigationGraph& get_graph()
  {
    return graph_;
  }

private:
  void update_edge(const graphnav::NavigationGraph::UUID& from, const graphnav::NavigationGraph::UUID& to,
                   const grid_map::GridMap& map, const Eigen::Transform<double, 3, Eigen::Isometry>& graph2map);

  graphnav::NavigationGraph graph_;
  std::optional<graphnav::NavigationGraph::UUID> current_node_;
  tf2_ros::Buffer& tf_buffer_;
  rclcpp::Logger logger_;

public:
  struct TraversabilityClass
  {
    std::string layer_name;
    double obstacle_threshold;
    double radius;
  };

  // parameters node sampling
  double node_spacing_ = 4.0;
  double node_free_radius_limit_ = 4.0;
  int traversability_update_nb_nodes_sample_ = 1000;
  double node_creation_hysteresis_ = 0.2;  // additional free radius required to create a new node

  // parameters edge building
  double edge_build_radius_ = 8.0;  // should be at least 2x node_spacing_
  double edge_creation_hysteresis_ = 0.15;  // additional free radius required to create a new edge
  std::map<std::string, TraversabilityClass> traversability_classes_ = {
    { "default", { "traversability", .99, 0.5 } },
  };
  std::string my_trav_class_ = "default";

  // parameters frontier building
  double explored_radius_margin_ = 0.75;
  double frontier_erase_unknown_dist_ = 0.3;
  double frontier_cell_spacing_ = 0.5;
  double frontier_cell_max_dist_ = 9.0;
  double frontier_end_margin_ = 0.0;
  double frontier_erase_visited_duration_ = 10.0;  // seconds, erase after being visited for this long
};

}  // namespace graphnav_mapper
