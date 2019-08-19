#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <week_slam/g2o/state_vertices.h>
#include <week_slam/g2o/edge/edge_landmark.h>
#include <week_slam/g2o/edge/edge_imu.h>
#include <week_slam/g2o/edge/edge_holonomic.h>
#include <week_slam/slam_node.h>

#include <ros/ros.h>

#include <pcl_ros/transforms.h>

SlamNode::SlamNode()
  : pnh_{ "~" }
  , params_{ pnh_ }
  , landmark_registration_{ params_.landmark_registration_options_, params_.barrel_ransac_options_ }
{
  pose_sub_ = nh_.subscribe("oswin/ground_truth", 1, &SlamNode::poseCallback, this);
  imu_sub_ = nh_.subscribe("oswin/imu", 1, &SlamNode::imuCallback, this);
  lidar_sub_ = nh_.subscribe("oswin/pointcloud", 1, &SlamNode::lidarCallback, this);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("oswin/odometry", 1);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("oswin/map", 1);
  landmark_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("oswin/landmarks", 1);

  mapper_.setInfo(200, 200, 0.1);

  pose_stamped_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  setupG2o();
  setupGraph();
  g2o::OptimizableGraph::initMultiThreading();
}

void SlamNode::setupG2o()
{
  auto linear_solver = std::make_unique<SlamLinearSolver>();
  linear_solver->setBlockOrdering(false);
  auto block_solver = std::make_unique<SlamBlockSolver>(std::move(linear_solver));
  auto* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(block_solver));

  optimizer_.setAlgorithm(solver);
}

void SlamNode::setupGraph()
{
  starting_time_ = ros::Time::now();
  g2o::StateVertices starting_state{};
  starting_state.allocate();
  starting_state.setIdsFrom(0);

  starting_state.se2->setFixed(true);
  starting_state.twist->setFixed(true);
  starting_state.stamp->setFixed(true);

  starting_state.insertIntoGraph(optimizer_);
}

void SlamNode::poseCallback(const geometry_msgs::PoseStamped& msg)
{
  pose_stamped_ = msg;
}

void SlamNode::imuCallback(const sensor_msgs::Imu& msg)
{
  addIMUEdge(msg);
  optimizeGraph();
  publishLatestPose();
}

// Hierarchical slam: Start with level 1
// When current node is some x distance away from last node, then take all new vertices (vector or something)
// and take like the middle node, insert it as a new vertex into the lower graph
// Take the last inserted vertex into the lower graph, calculate a relative transform from that to this,
// Unary edge for the twist.
//
// To do:
// 1: split up robotstate into SE2 and twist

void SlamNode::optimizeGraph()
{
  optimizer_.initializeOptimization();
  optimizer_.optimize(50);
}

void SlamNode::addIMUEdge(const sensor_msgs::Imu& msg)
{
  static ros::Time last_time = msg.header.stamp;
  static int last_id = -1;

  g2o::StateVertices last_imu_vertex;

  // Create first node
  if (last_id == -1)
  {
    ROS_INFO_STREAM("last_id == -1. Adding vertices with id starting from " << latest_pose_vertex_id_);
    last_imu_vertex = g2o::StateVertices(latest_pose_vertex_id_, optimizer_);
  }
  else
  {
    ROS_INFO_STREAM("last_id == " << last_id << ". Adding vertices with id starting from " << last_id);
    last_imu_vertex = g2o::StateVertices(last_id, optimizer_);
  }

  double delta_t = (msg.header.stamp - last_imu_vertex.stamp->estimate()).toSec();
  auto last_twist = last_imu_vertex.twist->estimate();
  auto last_se2 = last_imu_vertex.se2->estimate();

  g2o::StateVertices new_state_vertex;
  new_state_vertex.allocate();
  new_state_vertex.stamp->setEstimate(msg.header.stamp);
  new_state_vertex.twist->setEstimate(last_twist);
  new_state_vertex.se2->setEstimate(last_twist.apply(last_se2, delta_t));

  addVertex(new_state_vertex);
  last_id += 3;

  auto imu_edge = new g2o::EdgeIMU();
  g2o::IMUMeasurement measurement{ msg.linear_acceleration.x, msg.angular_velocity.z, delta_t };
  imu_edge->setMeasurement(measurement);
  imu_edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());

  last_imu_vertex.connectIMUEdge(new_state_vertex, *imu_edge);

  optimizer_.addEdge(imu_edge);
}

void SlamNode::addVertex(const g2o::StateVertices& new_vertices)
{
  int old_vertices_id = latest_pose_vertex_id_;
  int new_vertex_id = latest_vertex_id_ + 1;

  new_vertices.setIdsFrom(new_vertex_id);
  new_vertices.insertIntoGraph(optimizer_);

  g2o::StateVertices last_vertices(old_vertices_id, optimizer_);

  auto holonomic_constraint = new g2o::EdgeHolonomic();
  holonomic_constraint->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

  last_vertices.connectHolonomicEdge(new_vertices, *holonomic_constraint);
  optimizer_.addEdge(holonomic_constraint);

  latest_vertex_id_+=3;
  latest_pose_vertex_id_ = latest_vertex_id_ + 1;
}

void SlamNode::lidarCallback(const pcl::PointCloud<pcl::PointXYZ>& msg)
{
  tf::Transform transform_to_odom{};
  transform_to_odom.setOrigin({ pose_stamped_.pose.position.x, pose_stamped_.pose.position.y, 0.0 });
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose_stamped_.pose.orientation, quat);
  transform_to_odom.setRotation(quat);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud{};
  pcl_ros::transformPointCloud(msg, transformed_pointcloud, transform_to_odom);

  mapper_.addScan(transformed_pointcloud);
  //  ROS_INFO_STREAM("Starting <fn getLandmarks>");
  auto landmarks = landmark_registration_.getLandmarks(msg, transform_to_odom);
  auto id_comparator = [](const Landmark& a, const Landmark& b) { return a.id < b.id; };
  std::sort(landmarks.begin(), landmarks.end(), id_comparator);
  //  for (const auto& landmark : landmarks)
  //  {
  //    ROS_INFO_STREAM("[" << landmark.id << "] (" << landmark.barrel(0) << ", " << landmark.barrel(1) << ", "
  //                        << landmark.barrel(2) << ")");
  //  }
  //  ROS_INFO_STREAM("Publishing landmarks");

  publishLandmarks(landmarks);

  map_pub_.publish(mapper_.occupancy_grid_);

  addLandmarkEdges(landmarks);
  optimizeGraph();
  updateLandmarkLocations();
}

void SlamNode::updateLandmarkLocations()
{
  for (const auto [landmark_id, vertex_id] : landmark_map_)
  {
    auto landmark_vertex = static_cast<g2o::VertexPointXY*>(optimizer_.vertex(vertex_id));
    landmark_registration_.updateLandmark(landmark_id, landmark_vertex->estimate());
  }
}

void SlamNode::addLandmarkEdges(const std::vector<Landmark>& landmarks)
{
  for (const auto& landmark : landmarks)
  {
    if (auto entry = landmark_map_.find(landmark.id); entry != landmark_map_.end())
    {
      addLandmarkEdge(entry->second, landmark);
    }
    else
    {
      int landmark_vertex_id = addLandmarkVertex(landmark);
      addLandmarkEdge(landmark_vertex_id, landmark);
    }
  }
}

int SlamNode::addLandmarkVertex(const Landmark& landmark)
{
  int new_vertex_id = latest_vertex_id_ + 1;
  latest_vertex_id_++;

  auto landmark_vertex = new g2o::VertexPointXY{};
  landmark_vertex->setEstimate(landmark.barrel.head<2>());
  landmark_vertex->setId(new_vertex_id);

  optimizer_.addVertex(landmark_vertex);

  landmark_map_.emplace(std::make_pair(landmark.id, new_vertex_id));
  return new_vertex_id;
}

void SlamNode::addLandmarkEdge(int vertex_id, const Landmark& landmark)
{
  g2o::StateVertices last_vertices(latest_pose_vertex_id_, optimizer_);
  auto landmark_vertex = static_cast<g2o::VertexPointXY*>(optimizer_.vertex(vertex_id));
  // TODO(Oswin): Properly find the vertex

  auto* landmark_edge = new g2o::EdgeLandmark();
  landmark_edge->setMeasurement(landmark.barrel.head<2>());
  landmark_edge->setVertex(0, last_vertices.se2);
  landmark_edge->setVertex(1, landmark_vertex);
  landmark_edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());

  optimizer_.addEdge(landmark_edge);
}

void SlamNode::publishLandmarks(const std::vector<Landmark>& landmarks)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "oswin";

  for (const auto& landmark : landmarks)
  {
    pcl::PointXYZ point{ static_cast<float>(landmark.barrel(0)), static_cast<float>(landmark.barrel(1)),
                         static_cast<float>(landmark.id) };
    cloud.points.emplace_back(point);
  }
  landmark_pub_.publish(cloud);
}

void SlamNode::publishLatestPose()
{
  g2o::StateVertices latest_state(latest_pose_vertex_id_, optimizer_);

  auto& se2 = latest_state.se2->estimate();
  auto& twist = latest_state.twist->estimate();
  auto& stamp = latest_state.stamp->estimate();

  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(se2.rotation().angle());
  odometry.pose.pose.position.x = se2.translation()(0);
  odometry.pose.pose.position.y = se2.translation()(1);
  odometry.twist.twist.linear.x = twist.linear();
  odometry.twist.twist.angular.z = twist.angular();
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "odom";

  odom_pub_.publish(odometry);
}
