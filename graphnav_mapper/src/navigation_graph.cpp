#include "graphnav_mapper/navigation_graph.hpp"
#include <nanoflann.hpp>
#include <iostream>

namespace graphnav
{

void NavigationGraph::add_node(const Node& node)
{
  nodes[node.id] = node;
  nodes_not_in_kdtree_.push_back(node.id);
}

NavigationGraph::UUID NavigationGraph::add_node(const Eigen::Vector3d& position,
                                                const NavigationGraph::NodeProperties& properties)
{
  if (!pose_graph_.valid())
  {
    throw std::runtime_error("Pose graph is not set");
  }
  Node node;
  node.id = UUID::generate();
  node.pose_graph_node_id = pose_graph_.current_node();
  auto reference = pose_graph_.node_pose(node.pose_graph_node_id);
  assert(reference);  // current node is always in graph
  node.rel_position = reference->inverse() * position;
  node.properties = properties;
  add_node(node);
  return node.id;
}

void NavigationGraph::add_edge(const Edge& edge)
{
  edges[std::make_pair(edge.from, edge.to)] = edge;
  adjacency_[edge.from].insert(std::make_pair(edge.from, edge.to));
  adjacency_[edge.to].insert(std::make_pair(edge.from, edge.to));
}

void NavigationGraph::add_edge(const NavigationGraph::UUID& from, const NavigationGraph::UUID& to,
                               const NavigationGraph::EdgeProperties& properties)
{
  Edge edge;
  edge.from = from;
  edge.to = to;
  edge.properties = properties;
  add_edge(edge);
}

void NavigationGraph::remove_node(const NavigationGraph::UUID& id)
{
  for (const auto& edge : get_edges(id))
  {
    remove_edge(edge.first, edge.second);
  }
  nodes.erase(id);
}

void NavigationGraph::remove_edge(const NavigationGraph::UUID& from, const NavigationGraph::UUID& to)
{
  auto key = std::make_pair(from, to);
  edges.erase(key);
  adjacency_[from].erase(key);
  adjacency_[to].erase(key);
  if (adjacency_[from].empty())
  {
    adjacency_.erase(from);
  }
  if (adjacency_.find(to) != adjacency_.end() && adjacency_[to].empty())
  {
    adjacency_.erase(to);
  }
}

bool NavigationGraph::node_exists(const NavigationGraph::UUID& id) const
{
  return nodes.find(id) != nodes.end();
}

const NavigationGraph::Node& NavigationGraph::get_node(const NavigationGraph::UUID& id) const
{
  return nodes.at(id);
}

const NavigationGraph::NodeProperties& NavigationGraph::get_node_properties(const NavigationGraph::UUID& uuid) const
{
  return get_node(uuid).properties;
}

void NavigationGraph::set_node_properties(const NavigationGraph::UUID& uuid, const NodeProperties& properties)
{
  nodes[uuid].properties = properties;
}

Eigen::Isometry3d NavigationGraph::get_node_reference(const NavigationGraph::UUID& uuid) const
{
  const auto& node = get_node(uuid);
  auto reference = pose_graph_.node_pose(node.pose_graph_node_id);
  if (!reference)
  {
    throw std::runtime_error("Node pose not found in pose graph");
  }
  return *reference;
}

Eigen::Vector3d NavigationGraph::get_node_position(const NavigationGraph::UUID& uuid) const
{
  const auto& node = get_node(uuid);
  auto reference = pose_graph_.node_pose(node.pose_graph_node_id);
  if (!reference)
  {
    throw std::runtime_error("Node pose not found in pose graph");
  }
  return *reference * node.rel_position;
}

bool NavigationGraph::edge_exists(const NavigationGraph::UUID& from, const NavigationGraph::UUID& to) const
{
  return edges.find(std::make_pair(from, to)) != edges.end();
}

const NavigationGraph::EdgeProperties& NavigationGraph::get_edge_properties(const NavigationGraph::UUID& from,
                                                                            const NavigationGraph::UUID& to) const
{
  return edges.at(std::make_pair(from, to)).properties;
}

std::set<std::pair<NavigationGraph::UUID, NavigationGraph::UUID>>
NavigationGraph::get_edges(const NavigationGraph::UUID& node_id) const
{
  auto it = adjacency_.find(node_id);
  if (it == adjacency_.end())
  {
    return {};
  }
  return it->second;
}

void NavigationGraph::rebuild_kdtree() const
{
  kdtree_entries_.clear();
  nodes_not_in_kdtree_.clear();
  for (const auto& [id, node] : nodes)
  {
    auto reference = pose_graph_.node_pose(node.pose_graph_node_id);
    if (!reference)
    {
      continue;
    }
    Eigen::Vector3d world_pos = *reference * node.rel_position;
    kdtree_entries_.push_back({ id, world_pos });
  }
  kdtree_adaptor_ = std::make_unique<KDTreeAdaptor>(kdtree_entries_);
  nanoflann::KDTreeSingleIndexAdaptorParams params(30);
  kdtree_ = std::make_unique<KDTree>(3 /*dim*/, *kdtree_adaptor_, params);
  kdtree_->buildIndex();
}

std::vector<NavigationGraph::UUID> NavigationGraph::get_nodes_in_radius(const Eigen::Vector3d& position,
                                                                        double radius) const
{
  if (nodes_not_in_kdtree_.size() > kdtree_rebuild_threshold_)
  {
    rebuild_kdtree();
  }
  std::vector<UUID> result;
  if (kdtree_)
  {
#if NANOFLANN_VERSION >= 0x150  // 1.5.0
    std::vector<nanoflann::ResultItem<size_t, double>> idx_dist;
    nanoflann::SearchParameters params;
#else
    std::vector<std::pair<size_t, double>> idx_dist;
    nanoflann::SearchParams params;
#endif
    params.sorted = false;
    double point[3] = { position.x(), position.y(), position.z() };
    double radius_sq = radius * radius;
    kdtree_->radiusSearch(point, radius_sq, idx_dist, params);
    for (const auto& [idx, dist] : idx_dist)
    {
      result.push_back(kdtree_entries_[idx].id);
    }
  }
  // linear search for nodes not yet in kdtree
  for (const auto& node_id : nodes_not_in_kdtree_)
  {
    if (!node_exists(node_id))
    {
      continue;
    }
    Eigen::Vector3d node_pos = get_node_position(node_id);
    if ((node_pos - position).norm() < radius)
    {
      result.push_back(node_id);
    }
  }
  return result;
}

std::vector<NavigationGraph::UUID> NavigationGraph::get_all_nodes() const
{
  std::vector<UUID> result;
  for (const auto& node : nodes)
  {
    result.push_back(node.first);
  }
  return result;
}

void NavigationGraph::update_pose_graph(const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr pose_graph)
{
  pose_graph_.update(pose_graph);
  rebuild_kdtree();
}

std::optional<std::string> NavigationGraph::get_frame_id() const
{
  auto pg = pose_graph_.graph();
  if (!pg)
  {
    return std::nullopt;
  }
  return pg->header.frame_id;
}

graphnav_msgs::msg::NavigationGraph NavigationGraph::to_msg(const rclcpp::Time& stamp, const UUID& current_node) const
{
  graphnav_msgs::msg::NavigationGraph msg;
  msg.header.frame_id = *get_frame_id();
  msg.header.stamp = stamp;
  msg.trav_classes.clear();
  std::map<std::string, size_t> trav_class_indices;
  std::map<UUID, size_t> node_indices;
  for (const auto& [uuid, node] : nodes)
  {
    node_indices[node.id] = msg.nodes.size();
    graphnav_msgs::msg::Node node_msg;
    node_msg.uuid = node.id.to_msg();
    auto pg_reference = pose_graph_.node_pose(node.pose_graph_node_id);
    if (!pg_reference)
    {
      continue;
    }
    Eigen::Vector3d pos = *pg_reference * node.rel_position;
    node_msg.pose.position.x = pos.x();
    node_msg.pose.position.y = pos.y();
    node_msg.pose.position.z = pos.z();
    Eigen::Quaterniond orientation(pg_reference->rotation());
    node_msg.pose.orientation.x = orientation.x();
    node_msg.pose.orientation.y = orientation.y();
    node_msg.pose.orientation.z = orientation.z();
    node_msg.pose.orientation.w = orientation.w();
    node_msg.trav_properties.clear();
    for (const auto& [trav_class, trav_prop] : node.properties.trav_properties)
    {
      if (trav_class_indices.find(trav_class) == trav_class_indices.end())
      {  // new traversability class, add it
        trav_class_indices[trav_class] = msg.trav_classes.size();
        msg.trav_classes.push_back(trav_class);
      }
      size_t trav_class_idx = trav_class_indices.at(trav_class);
      if (node_msg.trav_properties.size() <= trav_class_idx)
      {
        node_msg.trav_properties.resize(trav_class_idx + 1);
      }
      node_msg.trav_properties[trav_class_idx].free_radius = trav_prop.free_radius;
      node_msg.trav_properties[trav_class_idx].is_frontier = trav_prop.is_frontier;
      node_msg.trav_properties[trav_class_idx].explored_radius = node.properties.explored_radius;
      if (trav_prop.is_frontier)
      {
        for (const auto& point : trav_prop.frontier_points)
        {
          geometry_msgs::msg::Point frontier_point;
          Eigen::Vector3d pos = *pg_reference * point;
          frontier_point.x = pos.x();
          frontier_point.y = pos.y();
          frontier_point.z = pos.z();
          node_msg.trav_properties[trav_class_idx].frontier_points.push_back(frontier_point);
        }
      }
    }
    msg.nodes.push_back(node_msg);
  }
  for (const auto& [uuid, edge] : edges)
  {
    size_t from_idx = node_indices.at(edge.from);
    size_t to_idx = node_indices.at(edge.to);
    graphnav_msgs::msg::Edge edge_msg;
    edge_msg.from_idx = from_idx;
    edge_msg.to_idx = to_idx;
    edge_msg.traversability.clear();
    for (const auto& [trav_class, trav_prop] : edge.properties.trav_properties)
    {
      size_t trav_class_idx = trav_class_indices.at(trav_class);
      if (edge_msg.traversability.size() <= trav_class_idx)
      {
        edge_msg.traversability.resize(trav_class_idx + 1);
      }
      edge_msg.traversability[trav_class_idx].traversability_cost = trav_prop.cost;
    }
    msg.edges.push_back(edge_msg);
  }
  msg.current_node_idx = node_indices.at(current_node);
  return msg;
}

visualization_msgs::msg::MarkerArray NavigationGraph::get_visualization(const rclcpp::Time& stamp,
                                                                        bool with_id_text) const
{
  visualization_msgs::msg::MarkerArray markers;

  // Marker for nodes
  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = *get_frame_id();
  node_marker.header.stamp = stamp;
  node_marker.frame_locked = true;
  node_marker.ns = "nodes";
  node_marker.id = 0;
  node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.scale.x = 0.2;  // Sphere radius
  node_marker.scale.y = 0.2;
  node_marker.scale.z = 0.2;
  node_marker.color.a = 1.0;  // Alpha
  node_marker.color.r = 0.0;
  node_marker.color.g = 1.0;
  node_marker.color.b = 0.0;

  // Marker for frontier nodes & points
  std::map<std::string, visualization_msgs::msg::Marker> frontier_node_markers;
  visualization_msgs::msg::Marker frontier_point_marker;
  frontier_point_marker.header.frame_id = *get_frame_id();
  frontier_point_marker.header.stamp = stamp;
  frontier_point_marker.frame_locked = true;
  frontier_point_marker.ns = "frontier_points";
  frontier_point_marker.id = 0;
  frontier_point_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  frontier_point_marker.action = visualization_msgs::msg::Marker::ADD;
  frontier_point_marker.scale.x = 0.1;
  frontier_point_marker.scale.y = 0.1;
  frontier_point_marker.scale.z = 0.1;
  frontier_point_marker.color.a = 1.0;
  frontier_point_marker.color.r = 1.0;
  frontier_point_marker.color.g = 0.0;
  frontier_point_marker.color.b = 1.0;
  visualization_msgs::msg::Marker frontier_point_to_node_marker;
  frontier_point_to_node_marker.header.frame_id = *get_frame_id();
  frontier_point_to_node_marker.header.stamp = stamp;
  frontier_point_to_node_marker.frame_locked = true;
  frontier_point_to_node_marker.ns = "frontier_point_to_node";
  frontier_point_to_node_marker.id = 0;
  frontier_point_to_node_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  frontier_point_to_node_marker.action = visualization_msgs::msg::Marker::ADD;
  frontier_point_to_node_marker.scale.x = 0.05;
  frontier_point_to_node_marker.color.a = 1.0;
  frontier_point_to_node_marker.color.b = 1.0;

  // Marker for free radius
  std::map<std::string, int> free_radius_counters;

  // Marker for explored radius
  int explored_radius_counter = 0;
  visualization_msgs::msg::Marker explored_radius_reset;
  explored_radius_reset.header.frame_id = *get_frame_id();
  explored_radius_reset.header.stamp = stamp;
  explored_radius_reset.ns = "explored_radius";
  explored_radius_reset.id = explored_radius_counter++;
  explored_radius_reset.action = visualization_msgs::msg::Marker::DELETEALL;

  // Marker for edges
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = *get_frame_id();
  edge_marker.header.stamp = stamp;
  edge_marker.frame_locked = true;
  edge_marker.ns = "edges";
  edge_marker.id = 1;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.scale.x = 0.05;  // Line width
  edge_marker.color.a = 1.0;
  edge_marker.color.r = 1.0;
  edge_marker.color.g = 0.0;
  edge_marker.color.b = 0.0;

  // Marker for relative positions
  visualization_msgs::msg::Marker rel_pos_marker;
  rel_pos_marker.header.frame_id = *get_frame_id();
  rel_pos_marker.header.stamp = stamp;
  rel_pos_marker.frame_locked = true;
  rel_pos_marker.ns = "relative_positions";
  rel_pos_marker.id = 2;
  rel_pos_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  rel_pos_marker.action = visualization_msgs::msg::Marker::ADD;
  rel_pos_marker.scale.x = 0.05;  // Line width
  rel_pos_marker.color.a = 1.0;
  rel_pos_marker.color.r = 0.0;
  rel_pos_marker.color.g = 0.0;
  rel_pos_marker.color.b = 1.0;

  for (const auto& node_pair : nodes)
  {
    const auto& node = node_pair.second;
    auto reference = pose_graph_.node_pose(node.pose_graph_node_id);
    if (!reference)
    {
      continue;
    }
    // std::cout << "node: " << (int)node.id.id[0]
    //     << " reference: " << (int)node.pose_graph_node_id.id[0]
    //     << " ref pos: " << reference->translation().transpose()
    //     << " rel pos: " << node.rel_position.transpose()
    //     << " pos: " << (*reference * node.rel_position).transpose()
    //     << std::endl;
    Eigen::Vector3d pos = *reference * node.rel_position;
    geometry_msgs::msg::Point node_position;
    node_position.x = pos.x();
    node_position.y = pos.y();
    node_position.z = pos.z();
    node_marker.points.push_back(node_position);

    // Add relative position line
    geometry_msgs::msg::Point ref_position;
    ref_position.x = reference->translation().x();
    ref_position.y = reference->translation().y();
    ref_position.z = reference->translation().z();
    rel_pos_marker.points.push_back(ref_position);
    rel_pos_marker.points.push_back(node_position);

    // Add explored radius
    visualization_msgs::msg::Marker explored_radius;
    explored_radius.header.frame_id = *get_frame_id();
    explored_radius.header.stamp = stamp;
    explored_radius.ns = "explored_radius";
    explored_radius.id = explored_radius_counter++;
    explored_radius.action = visualization_msgs::msg::Marker::ADD;
    explored_radius.type = visualization_msgs::msg::Marker::SPHERE;
    explored_radius.scale.x = node.properties.explored_radius * 2;
    explored_radius.scale.y = node.properties.explored_radius * 2;
    explored_radius.scale.z = 0.01;
    explored_radius.color.a = 0.2;
    explored_radius.color.r = 0.0;
    explored_radius.color.g = 0.0;
    explored_radius.color.b = 1.0;
    explored_radius.pose.position = node_position;
    markers.markers.push_back(explored_radius);

    // Add free radius for each traversability class
    for (const auto& [trav_class, trav_prop] : node.properties.trav_properties)
    {
      if (free_radius_counters.find(trav_class) == free_radius_counters.end())
      {
        free_radius_counters[trav_class] = 1;
        visualization_msgs::msg::Marker free_radius_reset;
        free_radius_reset.header.frame_id = *get_frame_id();
        free_radius_reset.header.stamp = stamp;
        free_radius_reset.ns = "free_radius_" + trav_class;
        free_radius_reset.id = 0;
        free_radius_reset.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(free_radius_reset);
      }
      visualization_msgs::msg::Marker free_radius;
      free_radius.header.frame_id = *get_frame_id();
      free_radius.header.stamp = stamp;
      free_radius.ns = "free_radius_" + trav_class;
      free_radius.id = free_radius_counters[trav_class]++;
      free_radius.action = visualization_msgs::msg::Marker::ADD;
      free_radius.type = visualization_msgs::msg::Marker::SPHERE;
      free_radius.scale.x = trav_prop.free_radius * 2;
      free_radius.scale.y = trav_prop.free_radius * 2;
      free_radius.scale.z = 0.01;
      free_radius.color.a = 0.2;
      free_radius.color.r = 1.0;
      free_radius.color.g = 0.0;
      free_radius.color.b = 0.0;
      free_radius.pose.position = node_position;
      markers.markers.push_back(free_radius);

      if (trav_prop.is_frontier || trav_prop.visited)
      {
        if (frontier_node_markers.find(trav_class) == frontier_node_markers.end())
        {
          frontier_node_markers[trav_class].header.frame_id = *get_frame_id();
          frontier_node_markers[trav_class].header.stamp = stamp;
          frontier_node_markers[trav_class].frame_locked = true;
          frontier_node_markers[trav_class].ns = "frontier_nodes_" + trav_class;
          frontier_node_markers[trav_class].id = 0;
          frontier_node_markers[trav_class].type = visualization_msgs::msg::Marker::SPHERE_LIST;
          frontier_node_markers[trav_class].action = visualization_msgs::msg::Marker::ADD;
          frontier_node_markers[trav_class].scale.x = 0.5;
          frontier_node_markers[trav_class].scale.y = 0.5;
          frontier_node_markers[trav_class].scale.z = 0.5;
          frontier_node_markers[trav_class].color.a = 1.0;
          frontier_node_markers[trav_class].color.b = 1.0;
        }
        frontier_node_markers[trav_class].points.push_back(node_position);
        auto color = std_msgs::msg::ColorRGBA();
        color.a = 1.0;
        if (trav_prop.visited)
        {
          color.r = 1.0;
        }
        if (trav_prop.is_frontier)
        {
          color.b = 1.0;
        }
        frontier_node_markers[trav_class].colors.push_back(color);
      }
      // add frontier points
      // TODO add to namespace by class
      for (const auto& point : trav_prop.frontier_points)
      {
        Eigen::Vector3d pos = *reference * point;
        geometry_msgs::msg::Point frontier_point;
        frontier_point.x = pos.x();
        frontier_point.y = pos.y();
        frontier_point.z = pos.z();
        frontier_point_marker.points.push_back(frontier_point);
        frontier_point_to_node_marker.points.push_back(node_position);
        frontier_point_to_node_marker.points.push_back(frontier_point);
      }
    }

  }

  for (const auto& edge_pair : edges)
  {
    const auto& edge = edge_pair.second;
    geometry_msgs::msg::Point from_position, to_position;
    Eigen::Vector3d from_pos = get_node_position(edge.from);
    Eigen::Vector3d to_pos = get_node_position(edge.to);
    from_position.x = from_pos.x();
    from_position.y = from_pos.y();
    from_position.z = from_pos.z();
    to_position.x = to_pos.x();
    to_position.y = to_pos.y();
    to_position.z = to_pos.z();
    edge_marker.points.push_back(from_position);
    edge_marker.points.push_back(to_position);
  }

  if (with_id_text)
  {
    visualization_msgs::msg::Marker text_marker_delete_all;
    text_marker_delete_all.header.frame_id = *get_frame_id();
    text_marker_delete_all.header.stamp = stamp;
    text_marker_delete_all.frame_locked = true;
    text_marker_delete_all.ns = "ids";
    text_marker_delete_all.id = 0;
    text_marker_delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(text_marker_delete_all);
    int id_counter = 1;
    for (const auto& node_pair : nodes)
    {
      const auto& node = node_pair.second;
      Eigen::Vector3d pos = get_node_position(node.id);
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = *get_frame_id();
      text_marker.header.stamp = stamp;
      text_marker.frame_locked = true;
      text_marker.ns = "ids";
      text_marker.id = id_counter++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.scale.z = 0.05;
      text_marker.text = node.id.to_string();
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.pose.position.x = pos.x();
      text_marker.pose.position.y = pos.y();
      text_marker.pose.position.z = pos.z() + 0.1;
      markers.markers.push_back(text_marker);
    }
  }

  markers.markers.push_back(node_marker);
  for (const auto& [trav_class, marker] : frontier_node_markers)
  {
    markers.markers.push_back(marker);
  }
  markers.markers.push_back(frontier_point_marker);
  markers.markers.push_back(frontier_point_to_node_marker);
  markers.markers.push_back(edge_marker);
  markers.markers.push_back(rel_pos_marker);

  return markers;
}

}  // namespace graphnav
