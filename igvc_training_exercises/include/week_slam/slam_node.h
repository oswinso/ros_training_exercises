#ifndef SRC_SLAM_NODE_H
#define SRC_SLAM_NODE_H

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>

#include <week_slam/slam_node_params.h>
#include <week_slam/landmark_registration.h>
#include <week_slam/mapper.h>
#include <week_slam/slam_node.h>

class SlamNode
{
 public:
  SlamNode();

 private:
  void poseCallback(const geometry_msgs::PoseStamped& msg);
  void lidarCallback(const pcl::PointCloud<pcl::PointXYZ>& msg);
  void publishLandmarks(const std::vector<Landmark>& landmarks);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber lidar_sub_;

  ros::Publisher map_pub_;
  ros::Publisher landmark_pub_;

  SlamNodeParams params_;
  LandmarkRegistration landmark_registration_;
  Mapper mapper_{};
  geometry_msgs::PoseStamped pose_stamped_{};
};

#endif //SRC_SLAM_NODE_H
