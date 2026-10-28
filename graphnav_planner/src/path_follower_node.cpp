#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace graphnav_planner
{

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode(const rclcpp::NodeOptions& options)
    : Node("path_follower_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_, this)
  {
    this->declare_parameter("wp_lookahead_dist", 2.0);
    wp_lookahead_dist_ = this->get_parameter("wp_lookahead_dist").as_double();
    this->declare_parameter("path_timeout", 1.0);
    path_timeout_ = this->get_parameter("path_timeout").as_double();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "~/path", 10, std::bind(&PathFollowerNode::path_callback, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/goal_pose", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, std::bind(&PathFollowerNode::odom_callback, this, std::placeholders::_1));
  }

private:
  void path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    path_ = msg;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!path_ || (rclcpp::Time(msg->header.stamp) - rclcpp::Time(path_->header.stamp)).seconds() > path_timeout_)
    {
      return;
    }
    if (path_->poses.empty())
    {
      return;
    }
    if (path_->poses.size() < 2)
    {
      pose_pub_->publish(path_->poses[0]);
      return;
    }
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;
    geometry_msgs::msg::PoseStamped odom_pose_in_path_frame;
    try
    {
      odom_pose_in_path_frame = tf_buffer_.transform(odom_pose, path_->header.frame_id, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform odometry pose to path frame: %s", ex.what());
      return;
    }
    Eigen::Vector3d robot_pos(odom_pose_in_path_frame.pose.position.x, odom_pose_in_path_frame.pose.position.y,
                              odom_pose_in_path_frame.pose.position.z);
    std::vector<Eigen::Vector3d> path_points;
    for (const auto& pose : path_->poses)
    {
      path_points.push_back(Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    }

    // Project the robot onto every path segment and start from the closest
    // continuous point. Node-only projection gets stuck when the lookahead
    // target lies on the same long segment.
    size_t closest_segment = 0;
    double closest_distance = std::numeric_limits<double>::max();
    Eigen::Vector3d closest_point = path_points.front();
    for (size_t index = 0; index + 1 < path_points.size(); ++index)
    {
      const Eigen::Vector3d start = path_points[index];
      const Eigen::Vector3d segment = path_points[index + 1] - start;
      const double length_squared = segment.squaredNorm();
      if (length_squared <= 1e-9)
      {
        continue;
      }
      const double ratio = std::clamp(
          (robot_pos - start).dot(segment) / length_squared, 0.0, 1.0);
      const Eigen::Vector3d projected = start + ratio * segment;
      const double distance = (projected - robot_pos).norm();
      if (distance < closest_distance)
      {
        closest_distance = distance;
        closest_segment = index;
        closest_point = projected;
      }
    }

    Eigen::Vector3d target = path_points.back();
    Eigen::Vector3d cursor = closest_point;
    double remaining = wp_lookahead_dist_;
    for (size_t index = closest_segment; index + 1 < path_points.size(); ++index)
    {
      const Eigen::Vector3d segment_end = path_points[index + 1];
      const Eigen::Vector3d segment = segment_end - cursor;
      const double segment_length = segment.norm();
      if (segment_length >= remaining)
      {
        target = cursor + segment / segment_length * remaining;
        break;
      }
      remaining -= segment_length;
      cursor = segment_end;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path_->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = target.x();
    pose.pose.position.y = target.y();
    pose.pose.position.z = target.z();
    pose.pose.orientation = path_->poses.back().pose.orientation;
    pose_pub_->publish(pose);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double wp_lookahead_dist_ = 2.0;
  double path_timeout_ = 1.0;

  nav_msgs::msg::Path::ConstSharedPtr path_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PathFollowerNode)
