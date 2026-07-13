#include "graphnav_mapper/mapper.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

namespace graphnav_mapper
{

Mapper::Mapper(tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger) : tf_buffer_(tf_buffer), logger_(logger)
{
}

void Mapper::odometry_update(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  rclcpp::Time now = odom->header.stamp;
  auto map_frame = graph_.get_frame_id();
  if (!map_frame)
  {
    RCLCPP_DEBUG(logger_, "no pose graph yet");
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom->header;
  pose.pose = odom->pose.pose;
  if (pose.header.frame_id != *map_frame)
  {
    try
    {
      tf_buffer_.transform(pose, pose, *map_frame, tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Transform lookuperror: %s", ex.what());
      return;
    }
  }
  Eigen::Vector3d odom_position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  if (!current_node_)
  {
    graphnav::NavigationGraph::NodeProperties properties;
    current_node_ = graph_.add_node(odom_position, properties);
    RCLCPP_DEBUG(logger_, "created initial node: %s", current_node_->to_string().c_str());
  }

  // current node can get deleted TODO this should be fixed better
  if (!graph_.node_exists(*current_node_))
  {
    RCLCPP_INFO(logger_, "current node %s deleted, creating new one", current_node_->to_string().c_str());
    graphnav::NavigationGraph::NodeProperties properties;
    properties.trav_properties[my_trav_class_].visited = true;
    properties.trav_properties[my_trav_class_].first_visited_time = now;
    current_node_ = graph_.add_node(odom_position, properties);
  }

  Eigen::Vector3d current_node_position = graph_.get_node_position(*current_node_);
  double current_node_distance = (current_node_position - odom_position).norm();
  auto nodes_in_radius = graph_.get_nodes_in_radius(odom_position, node_spacing_);
  // find closest node in radius
  std::optional<graphnav::NavigationGraph::UUID> closest_node;
  double closest_distance = std::numeric_limits<double>::max();
  for (const auto& node_id : nodes_in_radius)
  {
    if (node_id == *current_node_)
    {
      continue;
    }
    Eigen::Vector3d node_position = graph_.get_node_position(node_id);
    double distance = (node_position - odom_position).norm();
    if (distance < closest_distance)
    {  // TODO check that it is closer than the obstacle radius
      closest_distance = distance;
      closest_node = node_id;
    }
    // std::cout << "node: " << (int)node_id.id[0] << " distance: " << distance << std::endl;
  }
  // if closest node is closer than current node, switch to it
  if (closest_node && closest_distance < current_node_distance)
  {
    // add edge to closest node
    graphnav::NavigationGraph::EdgeProperties properties;  // TODO: add traversability
    if (graph_.edge_exists(*current_node_, *closest_node))
    {
      properties = graph_.get_edge_properties(*current_node_, *closest_node);
    }
    properties.trav_properties[my_trav_class_].traversed = true;
    graph_.add_edge(*current_node_, *closest_node, properties);
    current_node_ = closest_node;
    current_node_distance = closest_distance;
    RCLCPP_DEBUG(logger_, "switched to node: %s", current_node_->to_string().c_str());
    auto node_properties = graph_.get_node_properties(*current_node_);
    if (!node_properties.trav_properties[my_trav_class_].visited)
    {
      node_properties.trav_properties[my_trav_class_].first_visited_time = now;
      node_properties.trav_properties[my_trav_class_].visited = true;
    }
    graph_.set_node_properties(*current_node_, node_properties);
  }

  // create new node if far enough
  if (current_node_distance > node_spacing_)
  {
    // create new node
    graphnav::NavigationGraph::NodeProperties node_properties;
    node_properties.trav_properties[my_trav_class_].visited = true;
    node_properties.trav_properties[my_trav_class_].first_visited_time = now;
    auto new_node = graph_.add_node(odom_position, node_properties);
    graphnav::NavigationGraph::EdgeProperties edge_properties;  // TODO: add traversability
    graph_.add_edge(*current_node_, new_node, edge_properties);
    current_node_ = new_node;
    RCLCPP_DEBUG(logger_, "created new node: %s", current_node_->to_string().c_str());
  }

  RCLCPP_DEBUG(logger_, "Odometry updated");
}

visualization_msgs::msg::MarkerArray Mapper::get_visualization(rclcpp::Time stamp, bool with_id_text) const
{
  return graph_.get_visualization(stamp, with_id_text);
}

graphnav_msgs::msg::NavigationGraph Mapper::get_graph_msg(rclcpp::Time stamp) const
{
  if (!current_node_ || !graph_.node_exists(*current_node_))
  {
    graphnav_msgs::msg::NavigationGraph msg;
    msg.header.stamp = stamp;
    return msg;
  }
  return graph_.to_msg(stamp, *current_node_);
}

static void grid_map_set_border_unknown(grid_map::GridMap& gridmap, const std::string& layer_name)
{
  grid_map::Size size = gridmap.getSize();
  for (int x = 0; x < size(0); ++x)
  {
    gridmap.at(layer_name, grid_map::Index(x, 0)) = 1.0;            // Top border
    gridmap.at(layer_name, grid_map::Index(x, size(1) - 1)) = 1.0;  // Bottom border
  }
  for (int y = 0; y < size(1); ++y)
  {
    gridmap.at(layer_name, grid_map::Index(0, y)) = 1.0;            // Left border
    gridmap.at(layer_name, grid_map::Index(size(0) - 1, y)) = 1.0;  // Right border
  }
}

static void grid_map_create_sdf(grid_map::GridMap& gridmap, const std::string& layer_name, double threshold)
{
  cv::Mat in, sdf;
  Eigen::MatrixXf mat = gridmap.get(layer_name);
  // treat NaN as traversable / known
  mat = (mat.array().isFinite()).select(mat, static_cast<float>(threshold - 1e-3f));
  cv::eigen2cv(mat, in);
  // distanceTransform computes distance to zero inputs (condition false), so in > threshold
  cv::distanceTransform(in <= threshold, sdf, cv::DIST_L2, cv::DIST_MASK_5);
  sdf *= gridmap.getResolution();
  Eigen::MatrixXf out;
  cv::cv2eigen(sdf, out);
  gridmap.add(layer_name + "_sdf", out);
}

static void grid_map_apply_opening(grid_map::GridMap& gridmap, const std::string& layer_name, int kernel_size)
{
  cv::Mat in, out;
  cv::eigen2cv(gridmap.get(layer_name), in);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  cv::morphologyEx(in, out, cv::MORPH_OPEN, kernel);
  Eigen::MatrixXf out_eigen;
  cv::cv2eigen(out, out_eigen);
  gridmap.add(layer_name, out_eigen);
}

static std::vector<Eigen::Vector3d> grid_map_frontier_cells(grid_map::GridMap& gridmap,
                                                            const std::string& unknown_layer_name,
                                                            const std::string& elevation_layer_name)
{
  // auto start_time = std::chrono::high_resolution_clock::now();
  cv::Mat unknown;
  cv::eigen2cv(gridmap.get(unknown_layer_name), unknown);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(unknown <= 0, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  gridmap.add("frontier_dbg", 0.0);
  std::vector<Eigen::Vector3d> frontier_cells;
  for (const auto& contour : contours)
  {
    for (const auto& point : contour)
    {
      grid_map::Position3 pos;
      if (gridmap.getPosition3(elevation_layer_name, { point.y, point.x }, pos))
      {
        gridmap.atPosition("frontier_dbg", pos.head<2>()) = 1;
        frontier_cells.push_back(pos);
      }
    }
  }
  // double duration_ms =
  // std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() -
  // start_time).count() * 1000; std::cout << "Frontier cells found in " << duration_ms << " ms" << std::endl;
  return frontier_cells;
}

static bool collision_check_line(const Eigen::Vector2d& a, double sdf_a, const Eigen::Vector2d& b, double sdf_b,
                                 const grid_map::GridMap& map, std::string sdf_layer, double radius)
{
  if (sdf_a < radius || sdf_b < radius)
  {
    return true;
  }
  Eigen::Vector2d ab = b - a;
  double dist = ab.norm();
  if (dist <= map.getResolution())
  {
    return false;
  }
  if (dist < sqrt((sdf_a * sdf_a) - radius * radius) + sqrt((sdf_b * sdf_b) - radius * radius))
  {
    return false;
  }
  Eigen::Vector2d mid = (a + b) / 2;
  double sdf_mid = map.atPosition(sdf_layer, mid);
  return collision_check_line(a, sdf_a, mid, sdf_mid, map, sdf_layer, radius) ||
         collision_check_line(mid, sdf_mid, b, sdf_b, map, sdf_layer, radius);
}

static bool collision_check_line(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const grid_map::GridMap& map,
                                 std::string sdf_layer, double radius)
{
  double sdf_a = map.atPosition(sdf_layer, a);
  double sdf_b = map.atPosition(sdf_layer, b);
  return collision_check_line(a, sdf_a, b, sdf_b, map, sdf_layer, radius);
}

void Mapper::update_edge(const graphnav::NavigationGraph::UUID& from, const graphnav::NavigationGraph::UUID& to,
                         const grid_map::GridMap& map, const Eigen::Transform<double, 3, Eigen::Isometry>& graph2map)
{
  auto node_a = graph_.get_node_properties(from);
  Eigen::Vector2d pos_a = (graph2map * graph_.get_node_position(from)).head<2>();
  auto node_b = graph_.get_node_properties(to);
  Eigen::Vector2d pos_b = (graph2map * graph_.get_node_position(to)).head<2>();

  // Check that both positions are within the map bounds
  if (!map.isInside(pos_a) || !map.isInside(pos_b))
  {
    return;  // unknown, don't update or add edge
  }

  graphnav::NavigationGraph::EdgeProperties properties;
  if (graph_.edge_exists(from, to))
  {
    properties = graph_.get_edge_properties(from, to);
  }
  double unknown_sdf_a = map.atPosition("unknown_sdf", pos_a);
  double unknown_sdf_b = map.atPosition("unknown_sdf", pos_b);
  for (const auto& [trav_class, trav_prop] : traversability_classes_)
  {
    if (node_a.trav_properties.find(trav_class) == node_a.trav_properties.end() ||
        node_b.trav_properties.find(trav_class) == node_b.trav_properties.end())
    {
      continue;  // one of the nodes doesn't have this traversability class
    }
    double dist_a = node_a.trav_properties.at(trav_class).free_radius;
    double dist_b = node_b.trav_properties.at(trav_class).free_radius;
    double dist_ab = (pos_a - pos_b).norm();
    if (properties.trav_properties.find(trav_class) != properties.trav_properties.end() &&
        properties.trav_properties[trav_class].traversed)
    {
      properties.trav_properties[trav_class].cost = dist_ab;
      continue;  // already traversed, never erase
    }
    double r = trav_prop.radius;
    if (properties.trav_properties.find(trav_class) == properties.trav_properties.end())
    {
      // edge doesn't exist yet, require a bit more free space
      r += edge_creation_hysteresis_;
    }
    std::string sdf_layer = trav_prop.layer_name + "_sdf";
    if (!collision_check_line(pos_a, dist_a, pos_b, dist_b, map, sdf_layer, r))
    {
      if (collision_check_line(pos_a, unknown_sdf_a, pos_b, unknown_sdf_b, map, "unknown_sdf", r))
      {
        continue;  // unknown is too close to assess traversability, leave unchanged
      }
      properties.trav_properties[trav_class].cost = dist_ab;
    }
    else
    {  // collision, remove edge
      properties.trav_properties.erase(trav_class);
    }
  }

  if (properties.trav_properties.empty())
  {
    if (graph_.edge_exists(from, to))
    {
      // RCLCPP_INFO(logger_, "removing edge %s -> %s", from.to_string().c_str(), to.to_string().c_str());
      graph_.remove_edge(from, to);
    }
  }
  else
  {
    // RCLCPP_INFO(logger_, "adding edge %s -> %s", from.to_string().c_str(), to.to_string().c_str());
    graph_.add_edge(from, to, properties);
  }
}

void Mapper::traversability_update(const grid_map_msgs::msg::GridMap::ConstSharedPtr map,
                                   grid_map_msgs::msg::GridMap::SharedPtr dbg_map_out)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  auto graph_frame = graph_.get_frame_id();
  if (!graph_frame)
  {
    RCLCPP_DEBUG(logger_, "no pose graph yet");
    return;
  }
  Eigen::Transform<double, 3, Eigen::Isometry> graph2map;
  try
  {
    geometry_msgs::msg::TransformStamped transform;
    // get latest available transform, assuming both reference frames are ~static
    transform = tf_buffer_.lookupTransform(map->header.frame_id, *graph_frame, tf2::TimePointZero);
    graph2map = tf2::transformToEigen(transform);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Transform lookuperror: %s", ex.what());
    return;
  }
  Eigen::Transform<double, 3, Eigen::Isometry> map2graph = graph2map.inverse();
  rclcpp::Time now = map->header.stamp;

  auto start_time_gridmap = std::chrono::high_resolution_clock::now();
  std::vector<std::string> layer_names;
  for (const auto& [trav_class, trav_prop] : traversability_classes_)
  {
    layer_names.push_back(trav_prop.layer_name);
  }
  layer_names.push_back("unknown");
  layer_names.push_back("elevation");
  grid_map::GridMap gridmap;
  grid_map::GridMapRosConverter::fromMessage(*map, gridmap, layer_names, false, false);
  for (const auto& [trav_class, trav_prop] : traversability_classes_)
  {
    grid_map_create_sdf(gridmap, trav_prop.layer_name, trav_prop.obstacle_threshold);
  }
  grid_map_apply_opening(gridmap, "unknown", 3);
  grid_map_set_border_unknown(gridmap, "unknown");
  grid_map_create_sdf(gridmap, "unknown", 0.0);
  std::vector<Eigen::Vector3d> frontier_cells = grid_map_frontier_cells(gridmap, "unknown", "elevation");
  if (dbg_map_out)
  {
    *dbg_map_out = std::move(*grid_map::GridMapRosConverter::toMessage(gridmap));
  }
  auto duration_gridmap = std::chrono::duration_cast<std::chrono::duration<double>>(
                              std::chrono::high_resolution_clock::now() - start_time_gridmap)
                              .count() *
                          1000;
  RCLCPP_DEBUG(logger_, "gridmap update took %f ms", duration_gridmap);

  auto start_time_node_update = std::chrono::high_resolution_clock::now();
  std::vector<graphnav::NavigationGraph::UUID> nearby_nodes_in_map;
  Eigen::Vector2d map_center = gridmap.getPosition();
  double average_elevation = 0.0;
  if (!frontier_cells.empty()) {
    average_elevation = std::accumulate(frontier_cells.begin(), frontier_cells.end(), 0.0,
      [](double sum, const Eigen::Vector3d& cell) { return sum + cell.z(); }
    ) / frontier_cells.size();
  }
  Eigen::Vector3d map_center_graph =
      map2graph * Eigen::Vector3d(map->info.pose.position.x, map->info.pose.position.y, average_elevation);
  Eigen::Vector2d map_length(gridmap.getLength().x(), gridmap.getLength().y());
  nearby_nodes_in_map = graph_.get_nodes_in_radius(map_center_graph, map_length.norm() / 2);

  // update nearby node obstacle radii (per traversability class)
  for (const auto& node_id : nearby_nodes_in_map)
  {
    Eigen::Vector3d node_pos = graph_.get_node_position(node_id);
    Eigen::Vector2d node_pos_map = (graph2map * node_pos).head<2>();
    if (gridmap.isInside(node_pos_map))
    {
      double unknown_dist = gridmap.atPosition("unknown_sdf", node_pos_map);
      auto node_prop = graph_.get_node_properties(node_id);
      bool node_prop_changed = false;
      if (node_prop.explored_radius < unknown_dist)
      {
        node_prop.explored_radius = unknown_dist;  // only grow explored radius
        node_prop_changed = true;
      }
      for (const auto& [trav_class, trav_prop] : traversability_classes_)
      {
        double obstacle_dist = gridmap.atPosition(trav_prop.layer_name + "_sdf", node_pos_map);
        obstacle_dist = std::min(obstacle_dist, node_free_radius_limit_);
        double obst_or_unknown_dist = std::min(obstacle_dist, unknown_dist);
        if (node_prop.trav_properties.find(trav_class) == node_prop.trav_properties.end())
        {
          // node doesn't have this traversability class yet
          node_prop.trav_properties[trav_class].free_radius = obst_or_unknown_dist;
          node_prop_changed = true;
        }
        else
        {
          if (obstacle_dist < node_prop.trav_properties[trav_class].free_radius)
          {
            if (obstacle_dist > trav_prop.radius)
            {  // TODO hysteresis on creation / deletion radius
              node_prop.trav_properties[trav_class].free_radius = obstacle_dist;
              node_prop_changed = true;
            }
            else
            {
              // remove traversability class unless it's visited
              if (!node_prop.trav_properties[trav_class].visited)
              {
                node_prop.trav_properties.erase(trav_class);
                node_prop_changed = true;
              }
              continue;
            }
          }
          if (obst_or_unknown_dist > node_prop.trav_properties[trav_class].free_radius)
          {
            node_prop.trav_properties[trav_class].free_radius = obst_or_unknown_dist;
            node_prop_changed = true;
          }
        }
      }
      if (node_prop.trav_properties.empty())
      {
        RCLCPP_DEBUG(logger_, "removing node %s", node_id.to_string().c_str());
        graph_.remove_node(node_id);
      }
      else if (node_prop_changed)
      {
        RCLCPP_DEBUG(logger_, "updating node properties for %s", node_id.to_string().c_str());
        graph_.set_node_properties(node_id, node_prop);
      }
    }
  }
  auto duration_node_update = std::chrono::duration_cast<std::chrono::duration<double>>(
                                  std::chrono::high_resolution_clock::now() - start_time_node_update)
                                  .count() *
                              1000;
  RCLCPP_DEBUG(logger_, "node update took %f ms", duration_node_update);

  auto start_time_sample_new_nodes = std::chrono::high_resolution_clock::now();
  // sample new nodes
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(map_center.x() - map_length.x() / 2, map_center.x() + map_length.x() / 2);
  std::uniform_real_distribution<> dis_y(map_center.y() - map_length.y() / 2, map_center.y() + map_length.y() / 2);
  std::vector<graphnav::NavigationGraph::UUID> new_nodes;
  for (int i = 0; i < traversability_update_nb_nodes_sample_; ++i)
  {
    double x = dis_x(gen);
    double y = dis_y(gen);
    double z = map->info.pose.position.z + gridmap.atPosition("elevation", Eigen::Vector2d(x, y));
    if (!isfinite(z))
    {
      continue;
    }
    Eigen::Vector3d sample_pos_map(x, y, z);
    std::vector<std::pair<std::string, double>> free_class_radius_pairs;
    double sample_min_free_radius = std::numeric_limits<double>::max();
    double unknown_dist = gridmap.atPosition("unknown_sdf", sample_pos_map.head<2>());
    for (const auto& [trav_class, trav_prop] : traversability_classes_)
    {
      double obstacle_dist = gridmap.atPosition(trav_prop.layer_name + "_sdf", sample_pos_map.head<2>());
      double obst_or_unknown_dist = std::min(obstacle_dist, unknown_dist);
      double trav_free_radius = std::min(obst_or_unknown_dist, node_free_radius_limit_);
      if (trav_free_radius > trav_prop.radius + node_creation_hysteresis_)
      {
        sample_min_free_radius = std::min(sample_min_free_radius, trav_free_radius);
        free_class_radius_pairs.push_back(std::make_pair(trav_class, trav_free_radius));
      }
    }
    if (free_class_radius_pairs.empty())
    {
      continue;
    }
    Eigen::Vector3d sample_pos_graph = map2graph * sample_pos_map;
    bool close_to_existing_node = false;
    for (const auto& node_id : nearby_nodes_in_map)
    {
      if (!graph_.node_exists(node_id))
      {
        continue;
      }
      Eigen::Vector3d nearby_node_pos = graph_.get_node_position(node_id);
      double distance = (nearby_node_pos - sample_pos_graph).norm();
      double node_min_free_radius = std::numeric_limits<double>::max();
      auto node_prop = graph_.get_node_properties(node_id);
      for (const auto& [trav_class, trav_radius] : free_class_radius_pairs)
      {
        if (node_prop.trav_properties.find(trav_class) != node_prop.trav_properties.end())
        {
          node_min_free_radius = std::min(node_min_free_radius, node_prop.trav_properties[trav_class].free_radius);
        }
      }
      // TODO: we should instead check the min radii on either side over common traversability classes
      if (distance < node_min_free_radius || distance < sample_min_free_radius)
      {
        close_to_existing_node = true;
        break;
      }
    }
    if (close_to_existing_node)
    {
      continue;
    }
    graphnav::NavigationGraph::NodeProperties properties;
    for (const auto& [trav_class, trav_radius] : free_class_radius_pairs)
    {
      properties.trav_properties[trav_class].free_radius = trav_radius;
    }
    properties.explored_radius = unknown_dist;
    auto new_node = graph_.add_node(sample_pos_graph, properties);
    nearby_nodes_in_map.push_back(new_node);
    new_nodes.push_back(new_node);
    // RCLCPP_DEBUG(logger_, "sampled new node: %s", new_node.to_string().c_str());
  }
  auto duration_sample_new_nodes = std::chrono::duration_cast<std::chrono::duration<double>>(
                                       std::chrono::high_resolution_clock::now() - start_time_sample_new_nodes)
                                       .count() *
                                   1000;
  RCLCPP_DEBUG(logger_, "sample new nodes took %f ms", duration_sample_new_nodes);

  graph_.rebuild_kdtree();

  auto start_time_frontier_update = std::chrono::high_resolution_clock::now();
  // update frontier nodes
  struct extracted_frontier_properties
  {
    // cells that are part of the frontier (in graph frame) for each traversability class
    std::map<std::string, std::vector<Eigen::Vector3d>> frontier_cells;
  };
  std::map<graphnav::NavigationGraph::UUID, extracted_frontier_properties> extracted_frontiers;
  for (const auto& frontier_cell_pos_map : frontier_cells)
  {
    Eigen::Vector3d frontier_cell_pos_graph = map2graph * frontier_cell_pos_map;
    std::optional<graphnav::NavigationGraph::UUID> closest_node;
    double closest_distance = frontier_cell_max_dist_;
    std::set<std::string> closest_traversability_classes;
    for (const auto& node_id : graph_.get_nodes_in_radius(frontier_cell_pos_graph, frontier_cell_max_dist_))
    {
      Eigen::Vector3d node_pos = graph_.get_node_position(node_id);
      double distance = (node_pos - frontier_cell_pos_graph).norm();
      if (distance < graph_.get_node_properties(node_id).explored_radius - explored_radius_margin_)
      {
        closest_node.reset();  // already explored, ignore this frontier point
        break;
      }
      Eigen::Vector2d node_pos_map = (graph2map * node_pos).head<2>();
      if (!gridmap.isInside(node_pos_map))
      {
        continue;
      }
      bool node_is_new = std::find(new_nodes.begin(), new_nodes.end(), node_id) != new_nodes.end();
      if (distance < closest_distance)
      {
        std::set<std::string> traversability_classes;
        for (const auto& [trav_class, trav_prop] : traversability_classes_)
        {
          if (!collision_check_line(node_pos_map, frontier_cell_pos_map.head<2>(), gridmap,
                                    trav_prop.layer_name + "_sdf", trav_prop.radius))
          {
            auto node_trav_prop = graph_.get_node_properties(node_id).trav_properties;
            if (node_trav_prop[trav_class].is_frontier || node_is_new)
            {
              traversability_classes.insert(trav_class);
            }
          }
        }
        if (!traversability_classes.empty())
        {
          closest_distance = distance;
          closest_node = node_id;
          closest_traversability_classes = traversability_classes;
        }
      }
    }
    if (closest_node)
    {
      // frontier point is assigned to a node
      for (auto& trav_class : closest_traversability_classes)
      {
        auto node_properties = graph_.get_node_properties(*closest_node);
        if (node_properties.trav_properties[trav_class].visited)
        {
          continue;  // don't add frontiers to visited nodes
        }
        extracted_frontiers[*closest_node].frontier_cells[trav_class].push_back(frontier_cell_pos_graph);
      }
    }
  }
  auto duration_frontier_node_assignment = std::chrono::duration_cast<std::chrono::duration<double>>(
                                               std::chrono::high_resolution_clock::now() - start_time_frontier_update)
                                               .count() *
                                           1000;
  RCLCPP_DEBUG(logger_, "  frontier node assignment took %f ms", duration_frontier_node_assignment);
  auto start_time_frontier_clean_up = std::chrono::high_resolution_clock::now();
  // clean up explored frontiers
  for (const auto& node_id : nearby_nodes_in_map)
  {
    if (!graph_.node_exists(node_id))
    {
      continue;
    }
    auto node_properties = graph_.get_node_properties(node_id);
    Eigen::Isometry3d node2graph = graph_.get_node_reference(node_id);
    Eigen::Isometry3d node2map = graph2map * node2graph;
    bool node_properties_changed = false;
    for (auto& [trav_class, trav_property] : node_properties.trav_properties)
    {
      if (trav_property.frontier_points.empty())
      {
        continue;
      }
      if (trav_property.visited)
      {
        if (now - trav_property.first_visited_time > std::chrono::duration<double>(frontier_erase_visited_duration_))
        {
          trav_property.frontier_points.clear();
          trav_property.is_frontier = false;
          node_properties_changed = true;
          continue;
        }
      }
      size_t nb_frontier_points_before = trav_property.frontier_points.size();
      trav_property.frontier_points.erase(
          std::remove_if(trav_property.frontier_points.begin(), trav_property.frontier_points.end(),
                         [&gridmap, &node2map, this, &trav_class](const Eigen::Vector3d& point) {
                           Eigen::Vector2d point_map = (node2map * point).head<2>();
                           if (!gridmap.isInside(point_map))
                           {
                             return false;  // Keep the point if it's not inside the map
                           }
                           double unknown_dist = gridmap.atPosition("unknown_sdf", point_map);
                           if (unknown_dist > frontier_erase_unknown_dist_)
                           {
                             return true;  // Remove if in known area (unknown far away)
                           }
                           auto trav = traversability_classes_.at(trav_class);
                           double obstacle_dist = gridmap.atPosition(trav.layer_name + "_sdf", point_map);
                           if (obstacle_dist < trav.radius)
                           {
                             return true;  // Remove if in collision
                           }
                           return false;  // keep point
                         }),
          trav_property.frontier_points.end());
      if (trav_property.frontier_points.empty())
      {
        trav_property.is_frontier = false;
        node_properties_changed = true;
      }
      if (nb_frontier_points_before != trav_property.frontier_points.size())
      {
        node_properties_changed = true;
      }
    }
    if (node_properties_changed)
    {
      graph_.set_node_properties(node_id, node_properties);
    }
  }
  auto duration_frontier_clean_up = std::chrono::duration_cast<std::chrono::duration<double>>(
                                        std::chrono::high_resolution_clock::now() - start_time_frontier_clean_up)
                                        .count() *
                                    1000;
  RCLCPP_DEBUG(logger_, "  frontier clean up took %f ms", duration_frontier_clean_up);
  auto start_time_frontier_add_new = std::chrono::high_resolution_clock::now();
  // add new frontier points
  for (const auto& [node_id, frontier_properties] : extracted_frontiers)
  {
    auto node_properties = graph_.get_node_properties(node_id);
    bool node_properties_changed = false;
    Eigen::Isometry3d graph2node = graph_.get_node_reference(node_id).inverse();
    for (const auto& [trav_class, frontier_cells] : frontier_properties.frontier_cells)
    {
      for (const auto& frontier_cell : frontier_cells)
      {
        Eigen::Vector3d frontier_cell_rel = graph2node * frontier_cell;
        bool close_to_existing_point = false;
        for (const auto& existing_point : node_properties.trav_properties[trav_class].frontier_points)
        {
          if ((existing_point - frontier_cell_rel).norm() < frontier_cell_spacing_)
          {
            close_to_existing_point = true;
            break;
          }
        }
        if (!close_to_existing_point)
        {
          node_properties.trav_properties[trav_class].frontier_points.push_back(frontier_cell_rel);
          node_properties.trav_properties[trav_class].is_frontier = true;
          node_properties_changed = true;
        }
      }
    }
    if (node_properties_changed)
    {
      graph_.set_node_properties(node_id, node_properties);
    }
  }
  auto duration_frontier_add_new = std::chrono::duration_cast<std::chrono::duration<double>>(
                                       std::chrono::high_resolution_clock::now() - start_time_frontier_add_new)
                                       .count() *
                                   1000;
  RCLCPP_DEBUG(logger_, "  frontier add new took %f ms", duration_frontier_add_new);
  auto duration_frontier_update = std::chrono::duration_cast<std::chrono::duration<double>>(
                                      std::chrono::high_resolution_clock::now() - start_time_frontier_update)
                                      .count() *
                                  1000;
  RCLCPP_DEBUG(logger_, "frontier update took %f ms", duration_frontier_update);

  auto start_time_edge_update = std::chrono::high_resolution_clock::now();
  // update edges
  std::set<std::pair<graphnav::NavigationGraph::UUID, graphnav::NavigationGraph::UUID>> edges_to_check;
  // for all the nodes on map
  for (const auto& node_id : nearby_nodes_in_map)
  {
    if (!graph_.node_exists(node_id))
    {
      continue;
    }
    // all the edges of the node
    auto edges = graph_.get_edges(node_id);
    for (const auto& edge : edges)
    {
      edges_to_check.insert(edge);
    }
    // all the possible edges to other nodes in the radius
    auto others = graph_.get_nodes_in_radius(graph_.get_node_position(node_id), edge_build_radius_);
    for (const auto& other_id : others)
    {
      if (other_id == node_id)
      {
        continue;
      }
      edges_to_check.insert(std::make_pair(node_id, other_id));
    }
  }
  for (const auto& edge : edges_to_check)
  {
    // std::cout << "checking edge " << edge.first.to_string() << " -> " << edge.second.to_string() << std::endl;
    update_edge(edge.first, edge.second, gridmap, graph2map);
  }
  auto duration_edge_update = std::chrono::duration_cast<std::chrono::duration<double>>(
                                  std::chrono::high_resolution_clock::now() - start_time_edge_update)
                                  .count() *
                              1000;
  RCLCPP_DEBUG(logger_, "edge update took %f ms", duration_edge_update);

  double duration_ms =
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time)
          .count() *
      1000;
  RCLCPP_INFO(logger_, "Traversability updated in %f ms", duration_ms);
}

void Mapper::pose_graph_update(const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr pose_graph)
{
  graph_.update_pose_graph(pose_graph);
  RCLCPP_DEBUG(logger_, "Pose graph updated");
}

}  // namespace graphnav_mapper
