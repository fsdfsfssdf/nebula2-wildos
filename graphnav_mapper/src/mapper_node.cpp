#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "graphnav_mapper/mapper.hpp"

namespace graphnav_mapper
{

void occupancy_map_to_grid_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_map,
                               grid_map_msgs::msg::GridMap::SharedPtr grid_map)
{
  grid_map->header = occ_map->header;
  grid_map->info.resolution = occ_map->info.resolution;
  size_t size_x = occ_map->info.width;
  size_t size_y = occ_map->info.height;
  grid_map->info.length_x = size_x * occ_map->info.resolution;
  grid_map->info.length_y = size_y * occ_map->info.resolution;
  grid_map->info.pose.position.x = occ_map->info.origin.position.x + (size_x)*occ_map->info.resolution / 2.0;
  grid_map->info.pose.position.y = occ_map->info.origin.position.y + (size_y)*occ_map->info.resolution / 2.0;
  grid_map->info.pose.position.z = occ_map->info.origin.position.z;
  grid_map->info.pose.orientation.x = 0;
  grid_map->info.pose.orientation.y = 0;
  grid_map->info.pose.orientation.z = 0;
  grid_map->info.pose.orientation.w = 1;
  grid_map->layers = { "traversability", "unknown", "elevation" };
  grid_map->data.resize(3);
  grid_map->data[0].layout.data_offset = 0;
  grid_map->data[0].layout.dim.resize(2);
  grid_map->data[0].layout.dim[0].label = "column_index";
  grid_map->data[0].layout.dim[0].size = size_y;
  grid_map->data[0].layout.dim[0].stride = size_y * size_x;
  grid_map->data[0].layout.dim[1].label = "row_index";
  grid_map->data[0].layout.dim[1].size = size_x;
  grid_map->data[0].layout.dim[1].stride = size_x;
  grid_map->data[1].layout = grid_map->data[0].layout;
  grid_map->data[2].layout = grid_map->data[0].layout;
  grid_map->data[0].data.resize(size_x * size_y);
  grid_map->data[1].data.resize(size_x * size_y);
  grid_map->data[2].data.resize(size_x * size_y);
  for (size_t x = 0; x < size_x; x++)
  {
    for (size_t y = 0; y < size_y; y++)
    {
      size_t occ_map_index = y * size_x + x;
      size_t grid_map_index = (size_y - 1 - y) * size_x + (size_x - 1 - x);
      grid_map->data[0].data[grid_map_index] = std::clamp<int>(occ_map->data[occ_map_index], 0, 100) / 100.0;
      grid_map->data[1].data[grid_map_index] = occ_map->data[occ_map_index] == -1 ? 1.0 : 0.0;
      grid_map->data[2].data[grid_map_index] = 0.0;
    }
  }
}

class MapperNode : public rclcpp::Node
{
public:
  MapperNode(const rclcpp::NodeOptions& options)
    : Node("mapper_node", options)
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_, this)
    , mapper_(tf_buffer_, this->get_logger())
  {
    dbg_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("dbg_map", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          if (pose_graph_received_)
          {
            this->mapper_.odometry_update(msg);
          }
        });
    traversability_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "traversability", 10, std::bind(&MapperNode::traversability_cb, this, std::placeholders::_1));
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "occupancy", 10, [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          auto grid_map = std::make_shared<grid_map_msgs::msg::GridMap>();
          occupancy_map_to_grid_map(msg, grid_map);
          traversability_cb(grid_map);
        });
    pose_graph_sub_ = this->create_subscription<pose_graph_msgs::msg::PoseGraph>(
        "pose_graph", 10, [this](const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr msg) {
          try
          {
            this->mapper_.pose_graph_update(msg);
            pose_graph_received_ = true;
          }
          catch (const std::invalid_argument& e)
          {
            RCLCPP_ERROR(this->get_logger(), "Error updating pose graph: %s", e.what());
          }
        });
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("nav_graph_vis", 10);
    graph_pub_ = this->create_publisher<graphnav_msgs::msg::NavigationGraph>("nav_graph", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (pose_graph_received_)
      {
        auto markers = this->mapper_.get_visualization(this->get_clock()->now(), true);
        this->marker_pub_->publish(markers);
        auto graph = this->mapper_.get_graph_msg(this->get_clock()->now());
        this->graph_pub_->publish(graph);
      }
    });
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr traversability_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<pose_graph_msgs::msg::PoseGraph>::SharedPtr pose_graph_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr dbg_map_pub_;
  rclcpp::Publisher<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Mapper mapper_;
  bool pose_graph_received_ = false;

  void traversability_cb(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    if (pose_graph_received_)
    {
      std::shared_ptr<grid_map_msgs::msg::GridMap> dbg_map = nullptr;
      if (dbg_map_pub_->get_subscription_count() > 0)
      {
        dbg_map = std::make_shared<grid_map_msgs::msg::GridMap>();
      }
      this->mapper_.traversability_update(msg, dbg_map);
      if (dbg_map)
      {
        dbg_map_pub_->publish(*dbg_map);
      }
    }
  }
};

}  // namespace graphnav_mapper

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_mapper::MapperNode)
