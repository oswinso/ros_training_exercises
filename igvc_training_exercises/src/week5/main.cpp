#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

ros::Publisher g_map_pub;
nav_msgs::OccupancyGrid g_map;

void updateMap(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  // 1: Convert to odom frame, since data is in "oswin" frame
  static tf::TransformListener transform_listener;
  ros::Time stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);
  if (!transform_listener.waitForTransform("odom", "oswin", stamp, ros::Duration(5)))
  {
    ROS_ERROR_STREAM("Failed to find transform from odom to oswin.");
    return;
  }
  tf::StampedTransform transform_to_odom;
  transform_listener.lookupTransform("odom", "oswin", stamp, transform_to_odom);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl_ros::transformPointCloud(pointcloud, transformed_pointcloud, transform_to_odom);

  for (const auto& point : transformed_pointcloud)
  {
    int map_x = static_cast<int>(std::round((point.x - g_map.info.origin.position.x) / g_map.info.resolution));
    int map_y = static_cast<int>(std::round((point.y - g_map.info.origin.position.y) / g_map.info.resolution));
    int index = map_y * g_map.info.width + map_x;

    if (g_map.data[index] < 100)
    {
      g_map.data[index]++;
    }
  }
}

void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  g_map.header.stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);
  updateMap(pointcloud);
  g_map_pub.publish(g_map);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week5");

  ros::NodeHandle nh;

  g_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("oswin/map", 1);
  ros::Subscriber pointcloud_sub = nh.subscribe("oswin/pointcloud", 1, &pointcloudCallback);

  g_map.info.width = 200;
  g_map.info.height = 500;
  g_map.info.origin.position.x = -10;
  g_map.info.origin.position.y = -25;
  g_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
  g_map.info.resolution = 0.1;
  g_map.header.frame_id = "odom";

  int total_cells = g_map.info.width * g_map.info.height;

  g_map.data = std::vector<int8_t>(total_cells, 0);

  ros::spin();
}
