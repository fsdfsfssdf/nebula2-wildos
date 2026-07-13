#pragma once

#include <optional>
#include <pose_graph_msgs/msg/pose_graph.hpp>
#include <unordered_map>
#include <functional>
#include <array>
#include <Eigen/Dense>

// Define a hash function for pose_graph_msgs::msg::UUID needed for unordered_map
namespace std
{
template <>
struct hash<pose_graph_msgs::msg::UUID>
{
  std::size_t operator()(const pose_graph_msgs::msg::UUID& uuid) const noexcept
  {
    std::size_t hash = 0;
    for (const auto& byte : uuid.id)
    {
      hash = (hash << 1) ^ std::hash<uint8_t>{}(byte);
    }
    return hash;
  }
};
}  // namespace std

namespace graphnav
{

class PoseGraph
{
public:
  PoseGraph() : graph_(nullptr)
  {
  }

  bool valid() const
  {
    return graph_ != nullptr;
  }

  void update(const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr pose_graph)
  {
    if (pose_graph->nodes.empty())
    {
      throw std::invalid_argument("Pose graph is empty");
    }
    std::unordered_map<pose_graph_msgs::msg::UUID, const pose_graph_msgs::msg::Node*> map;
    bool current_node_found = false;
    for (const auto& node : pose_graph->nodes)
    {
      map[node.uuid] = &node;
      if (node.uuid == pose_graph->current_node)
      {
        current_node_found = true;
      }
    }
    if (!current_node_found)
    {
      throw std::invalid_argument("Current node not in pose graph");
    }
    graph_ = pose_graph;
    nodes_ = std::move(map);
  }

  const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr graph() const
  {
    return graph_;
  }
  pose_graph_msgs::msg::UUID current_node() const
  {
    return graph_->current_node;
  }
  std::optional<Eigen::Transform<double, 3, Eigen::Isometry>>
  node_pose(const pose_graph_msgs::msg::UUID& node_uuid) const
  {
    if (nodes_.find(node_uuid) == nodes_.end())
    {
      return std::nullopt;
    }
    const auto& node = nodes_.at(node_uuid);
    Eigen::Transform<double, 3, Eigen::Isometry> transform;
    transform.translation() = Eigen::Vector3d(node->pose.position.x, node->pose.position.y, node->pose.position.z);
    Eigen::Quaterniond q(node->pose.orientation.w, node->pose.orientation.x, node->pose.orientation.y,
                         node->pose.orientation.z);
    transform.linear() = q.toRotationMatrix();
    return transform;
  }

private:
  pose_graph_msgs::msg::PoseGraph::ConstSharedPtr graph_;
  std::unordered_map<pose_graph_msgs::msg::UUID, const pose_graph_msgs::msg::Node*> nodes_;
};

}  // namespace graphnav
