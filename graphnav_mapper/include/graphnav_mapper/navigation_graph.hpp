#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <functional>
#include <random>
#include <Eigen/Dense>
#include <nanoflann.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <graphnav_msgs/msg/navigation_graph.hpp>
#include <graphnav_msgs/msg/uuid.hpp>
#include "graphnav_mapper/pose_graph.hpp"

namespace graphnav
{

class NavigationGraph
{
public:
  struct UUID
  {
    uint8_t id[16];

    static UUID generate()
    {
      UUID uuid;
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, 255);
      for (auto& byte : uuid.id)
      {
        byte = static_cast<uint8_t>(dis(gen));
      }
      return uuid;
    }

    bool operator<(const UUID& other) const
    {
      return std::memcmp(id, other.id, sizeof(id)) < 0;
    }

    bool operator==(const UUID& other) const
    {
      return std::memcmp(id, other.id, sizeof(id)) == 0;
    }

    std::string to_string() const
    {
      std::ostringstream oss;
      oss << std::hex << std::setfill('0');
      for (int i = 0; i < 16; ++i)
      {
        if (i == 4 || i == 6 || i == 8 || i == 10)
        {
          oss << '-';
        }
        oss << std::setw(2) << static_cast<int>(id[i]);
      }
      return oss.str();
    }

    graphnav_msgs::msg::UUID to_msg() const
    {
      graphnav_msgs::msg::UUID msg;
      std::memcpy(msg.id.data(), id, sizeof(id));
      return msg;
    }
  };

  struct NodeProperties
  {
    struct trav_properties_t
    {
      bool is_frontier = false;
      double free_radius = 0.0;  // used for sampling density
      std::vector<Eigen::Vector3d> frontier_points;
      bool visited = false;
      rclcpp::Time first_visited_time;
    };
    std::map<std::string, trav_properties_t> trav_properties;
    double explored_radius = 0.0;
  };

  struct Node
  {
    UUID id;
    pose_graph_msgs::msg::UUID pose_graph_node_id;
    Eigen::Vector3d rel_position;
    NodeProperties properties;
  };

  struct EdgeProperties
  {
    struct trav_properties_t
    {
      float cost = std::numeric_limits<float>::infinity();
      bool traversed = false;
    };
    std::map<std::string, trav_properties_t> trav_properties;
  };

  struct Edge
  {
    UUID from;
    UUID to;
    EdgeProperties properties;
  };

  NavigationGraph()
  {
  }
  void add_node(const Node& node);
  UUID add_node(const Eigen::Vector3d& position, const NodeProperties& properties);
  void add_edge(const Edge& edge);
  void add_edge(const UUID& from, const UUID& to, const EdgeProperties& properties);
  void remove_node(const UUID& id);
  void remove_edge(const UUID& from, const UUID& to);
  bool node_exists(const UUID& id) const;
  const Node& get_node(const UUID& id) const;
  const NodeProperties& get_node_properties(const UUID& uuid) const;
  void set_node_properties(const UUID& uuid, const NodeProperties& properties);
  Eigen::Isometry3d get_node_reference(const UUID& uuid) const;
  Eigen::Vector3d get_node_position(const UUID& uuid) const;
  bool edge_exists(const UUID& from, const UUID& to) const;
  const EdgeProperties& get_edge_properties(const UUID& from, const UUID& to) const;
  std::set<std::pair<UUID, UUID>> get_edges(const UUID& node_id) const;
  std::vector<UUID> get_nodes_in_radius(const Eigen::Vector3d& position, double radius) const;
  std::vector<UUID> get_all_nodes() const;
  void update_pose_graph(const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr pose_graph);
  void rebuild_kdtree() const;
  std::optional<std::string> get_frame_id() const;
  graphnav_msgs::msg::NavigationGraph to_msg(const rclcpp::Time& stamp, const UUID& current_node) const;
  visualization_msgs::msg::MarkerArray get_visualization(const rclcpp::Time& stamp, bool with_id_text = false) const;

private:
  struct UUIDHash
  {
    std::size_t operator()(const UUID& uuid) const noexcept
    {
      std::size_t hash_value = 0;
      for (const auto& byte : uuid.id)
      {
        hash_value = (hash_value << 1) ^ std::hash<uint8_t>{}(byte);
      }
      return hash_value;
    }
  };
  struct UUIDPairHash
  {
    std::size_t operator()(const std::pair<UUID, UUID>& pair) const noexcept
    {
      auto h1 = UUIDHash{}(pair.first);
      auto h2 = UUIDHash{}(pair.second);
      return h1 ^ (h2 << 1);
    }
  };

  struct KDTreeEntry
  {
    UUID id;
    Eigen::Vector3d world_position;
  };

  class KDTreeAdaptor
  {
  public:
    KDTreeAdaptor(const std::vector<KDTreeEntry>& entries) : entries_(entries)
    {
    }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
      if (dim == 0)
        return entries_[idx].world_position.x();
      if (dim == 1)
        return entries_[idx].world_position.y();
      if (dim == 2)
        return entries_[idx].world_position.z();
      return 0;
    }
    inline size_t kdtree_get_point_count() const
    {
      return entries_.size();
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
      return false;
    }

  private:
    const std::vector<KDTreeEntry>& entries_;
  };

  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, KDTreeAdaptor>, KDTreeAdaptor,
                                                     3, size_t>;

  std::unordered_map<UUID, Node, UUIDHash> nodes;
  std::unordered_map<std::pair<UUID, UUID>, Edge, UUIDPairHash> edges;
  std::unordered_map<UUID, std::set<std::pair<UUID, UUID>>, UUIDHash> adjacency_;

  mutable std::vector<KDTreeEntry> kdtree_entries_;
  mutable std::vector<UUID> nodes_not_in_kdtree_;
  mutable std::unique_ptr<KDTreeAdaptor> kdtree_adaptor_;
  mutable std::unique_ptr<KDTree> kdtree_;
  static constexpr size_t kdtree_rebuild_threshold_ = 30;

  graphnav::PoseGraph pose_graph_;
};

}  // namespace graphnav
