#include <gtest/gtest.h>
#include "graphnav_mapper/mapper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "pose_graph_msgs/msg/pose_graph.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

static bool publish_on_failure_ = false;

class MapperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::Logger logger = rclcpp::get_logger("MapperTest");
    logger.set_level(rclcpp::Logger::Level::Debug);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    mapper_ = std::make_shared<graphnav_mapper::Mapper>(*tf_buffer_, logger);

    mapper_->traversability_classes_["default"].layer_name = "traversability";
    mapper_->traversability_classes_["default"].obstacle_threshold = 0.001;
    mapper_->traversability_classes_["default"].radius = 1.0;

    stamp_ = rclcpp::Time(1, 0, RCL_ROS_TIME);

    grid_map_.setFrameId("map");
    grid_map_.setGeometry(grid_map::Length(50.0, 50.0), 0.1, grid_map::Position(10.0, 5.0));
    grid_map_.add("unknown", 0.0);
    grid_map_.add("traversability", 0.0);
    grid_map_.add("elevation", 0.0);
    grid_map_.setTimestamp(stamp_.nanoseconds());
    dbg_map_ = nullptr;

    pose_graph_ = std::make_shared<pose_graph_msgs::msg::PoseGraph>();
    pose_graph_->header.frame_id = "map";
    pose_graph_->header.stamp = stamp_;
    pose_graph_->nodes.resize(1);
    pose_graph_->nodes[0].pose.orientation.w = 1.0;
    pose_graph_->nodes[0].uuid.id[0] = 1;
    pose_graph_->current_node = pose_graph_->nodes[0].uuid;
    pose_graph_->edges.resize(0);

    mapper_->pose_graph_update(pose_graph_);
  }

  void TearDown() override
  {
    if (::testing::Test::HasFailure() && publish_on_failure_)
    {
      std::cout << "Test failed, publishing debug information..." << std::endl;
      std::cout << "Test name: " << ::testing::UnitTest::GetInstance()->current_test_info()->name() << std::endl;
      auto node = rclcpp::Node::make_shared("test_node");
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.transient_local();
      auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>("debug_map", qos);
      if (dbg_map_)
      {
        publisher->publish(*dbg_map_);
      }
      auto graph_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("graph", qos);
      graph_publisher->publish(mapper_->get_visualization(stamp_, true));
      std::cout << "Press Enter to continue..." << std::endl;
      std::cin.get();
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<graphnav_mapper::Mapper> mapper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  grid_map::GridMap grid_map_;
  std::shared_ptr<grid_map_msgs::msg::GridMap> dbg_map_;
  std::shared_ptr<pose_graph_msgs::msg::PoseGraph> pose_graph_;
  rclcpp::Time stamp_;
};

TEST_F(MapperTest, OdometryUpdateTest)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->pose.pose.position.x = 1.0;
  odom->pose.pose.position.y = 2.0;
  odom->pose.pose.orientation.w = 1.0;

  mapper_->odometry_update(odom);

  // Add assertions to verify the state of the mapper after the odometry update
  // For example, check if the current node is set correctly
  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdateCreatesNodesInFreeSpace)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10)
    {
      grid_map_.at("traversability", *it) = 1.0;
    }
  }
  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update(msg, dbg_map_);

  // ASSERT_TRUE(false);

  auto nodes = mapper_->get_graph().get_all_nodes();
  EXPECT_GT(nodes.size(), 0);  // nodes are created
  for (const auto& node : nodes)
  {
    auto pos = mapper_->get_graph().get_node_position(node);
    EXPECT_TRUE(grid_map_.isInside(pos.head<2>()));
    EXPECT_LT(grid_map_.atPosition("unknown", pos.head<2>()), 0.5);
    EXPECT_LT(grid_map_.atPosition("traversability", pos.head<2>()), 0.5);
  }

  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdatesNodeProperties)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10)
    {
      grid_map_.at("traversability", *it) = 1.0;
    }
  }

  // add a node in the free space
  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties init_properties;
  init_properties.trav_properties["default"].free_radius = mapper_->node_free_radius_limit_;
  init_properties.explored_radius = 2.0;
  auto node_to_update = graph.add_node(Eigen::Vector3d(5.0, 8.0, 0.0), init_properties);
  auto node_to_delete = graph.add_node(Eigen::Vector3d(15.0, 15.0, 0.0), init_properties);
  auto node_to_leave = graph.add_node(Eigen::Vector3d(19.0, 0.0, 0.0), init_properties);
  // std::cout << "node_to_update: " << node_to_update.to_string() << std::endl;
  // std::cout << "node_to_delete: " << node_to_delete.to_string() << std::endl;
  // std::cout << "node_to_leave: " << node_to_leave.to_string() << std::endl;

  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update(msg, dbg_map_);

  // updated free radius & explored radius
  EXPECT_TRUE(graph.node_exists(node_to_update));
  auto node_prop = graph.get_node_properties(node_to_update);
  EXPECT_EQ(node_prop.trav_properties.size(), 1);
  EXPECT_NEAR(node_prop.trav_properties["default"].free_radius, 10 - 8, 0.2);
  EXPECT_NEAR(node_prop.explored_radius, 20 - 5, 0.2);

  // unchanged node
  EXPECT_TRUE(graph.node_exists(node_to_leave));
  node_prop = graph.get_node_properties(node_to_leave);
  EXPECT_EQ(node_prop.trav_properties.size(), 1);
  EXPECT_NEAR(node_prop.trav_properties["default"].free_radius, init_properties.trav_properties["default"].free_radius,
              0.01);
  EXPECT_NEAR(node_prop.explored_radius, init_properties.explored_radius, 0.01);

  // deleted node (too close to obstacle)
  EXPECT_FALSE(graph.node_exists(node_to_delete));

  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdatesEdges)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10 && position.y() < 11)
    {
      grid_map_.at("traversability", *it) = 1.0;
    }
  }

  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties init_properties;
  init_properties.trav_properties["default"].free_radius = 1.0;
  init_properties.explored_radius = 2.0;
  graphnav::NavigationGraph::EdgeProperties edge_properties;
  edge_properties.trav_properties["default"].cost = 1.0;
  // far from obstacle or unknown, should create edge
  auto node_free_a = graph.add_node(Eigen::Vector3d(5.0, 0.0, 0.0), init_properties);
  auto node_free_b = graph.add_node(Eigen::Vector3d(0.0, 0.0, 0.0), init_properties);
  // across obstacle, should remove edge
  auto node_obs_a = graph.add_node(Eigen::Vector3d(0.0, 8.0, 0.0), init_properties);
  auto node_obs_b = graph.add_node(Eigen::Vector3d(0.0, 13.0, 0.0), init_properties);
  graph.add_edge(node_obs_a, node_obs_b, edge_properties);
  // too close to unknown, should not create edge
  auto node_unknown_a = graph.add_node(Eigen::Vector3d(19.5, 0.0, 0.0), init_properties);
  auto node_unknown_b = graph.add_node(Eigen::Vector3d(19.5, 2.0, 0.0), init_properties);
  // close/in unknown, should preserve edge
  auto node_unknown_keep_a = graph.add_node(Eigen::Vector3d(25, 0.0, 0.0), init_properties);
  auto node_unknown_keep_b = graph.add_node(Eigen::Vector3d(25, 2.0, 0.0), init_properties);
  graph.add_edge(node_unknown_keep_a, node_unknown_keep_b, edge_properties);
  // close to unknown, but across obstacle, should remove edge
  auto node_unknown_obs_a = graph.add_node(Eigen::Vector3d(19.5, 8.0, 0.0), init_properties);
  auto node_unknown_obs_b = graph.add_node(Eigen::Vector3d(19.5, 13.0, 0.0), init_properties);
  graph.add_edge(node_unknown_obs_a, node_unknown_obs_b, edge_properties);

  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update_nb_nodes_sample_ = 0;  // don't sample new nodes
  mapper_->traversability_update(msg, dbg_map_);

  EXPECT_TRUE(graph.edge_exists(node_free_a, node_free_b));
  EXPECT_FALSE(graph.edge_exists(node_obs_a, node_obs_b));
  EXPECT_FALSE(graph.edge_exists(node_unknown_a, node_unknown_b));
  EXPECT_TRUE(graph.edge_exists(node_unknown_keep_a, node_unknown_keep_b));
  EXPECT_FALSE(graph.edge_exists(node_unknown_obs_a, node_unknown_obs_b));

  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdateCreatesFrontiers)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10)
    {
      grid_map_.at("traversability", *it) = 1.0;
      if (position.y() > 10.2)
      {  // unknown behind obstacle
        grid_map_.at("unknown", *it) = 1.0;
      }
    }
  }

  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->frontier_cell_max_dist_ = 20.0;  // large enough to get all frontiers TODO a bit hacky
  mapper_->traversability_update(msg, dbg_map_);

  double coll_radius = mapper_->traversability_classes_["default"].radius;
  double map_y_min = grid_map_.getPosition().y() - grid_map_.getLength().y() / 2;
  double map_x_min = grid_map_.getPosition().x() - grid_map_.getLength().x() / 2;
  std::vector<Eigen::Vector2d> frontiers;
  std::vector<bool> frontier_found;
  auto& graph = mapper_->get_graph();
  for (const auto& node : graph.get_all_nodes())
  {
    auto node_properties = graph.get_node_properties(node);
    for (const auto& frontier : node_properties.trav_properties["default"].frontier_points)
    {
      frontiers.push_back(frontier.head<2>());
    }
  }
  frontier_found.resize(frontiers.size(), false);
  double tol = 0.5;

  // frontiers along unknown
  // x = 20, y in [map_y_min, 10 - radius]
  for (double y = map_y_min; y < 10 - coll_radius; y += 0.1)
  {
    auto pos = Eigen::Vector2d(20, y);
    bool found_match = false;
    for (size_t i = 0; i < frontiers.size(); i++)
    {
      if ((frontiers[i] - pos).norm() < tol)
      {
        frontier_found[i] = true;
        found_match = true;
      }
    }
    ASSERT_TRUE(found_match) << "No frontier found at position " << pos.transpose();
  }

  // frontiers along map boundaries
  // x in [map_x_min, 20], y = map_y_min
  // x = map_x_min, y in [map_y_min, 10 - radius]
  for (double x = map_x_min; x < 20; x += 0.1)
  {
    auto pos = Eigen::Vector2d(x, map_y_min);
    bool found_match = false;
    for (size_t i = 0; i < frontiers.size(); i++)
    {
      if ((frontiers[i] - pos).norm() < tol)
      {
        frontier_found[i] = true;
        found_match = true;
      }
    }
    ASSERT_TRUE(found_match) << "No frontier found at position " << pos.transpose();
  }
  for (double y = map_y_min; y < 10 - coll_radius; y += 0.1)
  {
    auto pos = Eigen::Vector2d(map_x_min, y);
    bool found_match = false;
    for (size_t i = 0; i < frontiers.size(); i++)
    {
      if ((frontiers[i] - pos).norm() < tol)
      {
        frontier_found[i] = true;
        found_match = true;
      }
    }
    ASSERT_TRUE(found_match) << "No frontier found at position " << pos.transpose();
  }

  // should have no frontiers along obstacle
  // x in [map_x_min, 20], y = 10
  for (double x = map_x_min; x < 20; x += 0.1)
  {
    auto pos = Eigen::Vector2d(x, 10);
    ASSERT_TRUE(std::find_if(frontiers.begin(), frontiers.end(),
                             [&](const Eigen::Vector2d& frontier) { return (frontier - pos).norm() < tol; }) ==
                frontiers.end())
        << "Frontier found at position " << pos.transpose();
  }

  for (size_t i = 0; i < frontiers.size(); i++)
  {
    ASSERT_TRUE(frontier_found[i]) << "Frontier at position " << frontiers[i].transpose()
                                   << " was not found in expected locations";
  }

  // ASSERT_TRUE(false);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  if (std::find_if(argv, argv + argc, [](const char* arg) { return std::string(arg) == "--publish-on-failure"; }) !=
      argv + argc)
  {
    publish_on_failure_ = true;
  }
  return RUN_ALL_TESTS();
}
