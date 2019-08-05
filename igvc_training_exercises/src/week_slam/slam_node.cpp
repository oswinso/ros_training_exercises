#include <week_slam/slam_node.h>

#include <ros/ros.h>

#include <pcl_ros/transforms.h>

SlamNode::SlamNode() : pnh_{ "~" }, params_{ pnh_ }, landmark_registration_{ params_.landmark_registration_options_, params_.barrel_ransac_options_ }
{
  pose_sub_ = nh_.subscribe("oswin/ground_truth", 1, &SlamNode::poseCallback, this);
  lidar_sub_ = nh_.subscribe("oswin/pointcloud", 1, &SlamNode::lidarCallback, this);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("oswin/map", 1);
  landmark_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("oswin/landmarks", 1);

  mapper_.setInfo(200, 200, 0.1);

  pose_stamped_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

void SlamNode::poseCallback(const geometry_msgs::PoseStamped& msg)
{
  pose_stamped_ = msg;
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
}

void SlamNode::publishLandmarks(const std::vector<Landmark>& landmarks)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "odom";

  for (const auto& landmark : landmarks)
  {
    pcl::PointXYZ point{static_cast<float>(landmark.barrel(0)), static_cast<float>(landmark.barrel(1)), static_cast<float>(landmark.id)};
    cloud.points.emplace_back(point);
  }
  landmark_pub_.publish(cloud);
}
