#include <ros/ros.h>

#include <week_scanmatching/icp/icp.h>

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Affine3d& transformation)
{
  for (auto& point : cloud)
  {
    point.getVector3fMap() = transformation.cast<float>() * point.getVector3fMap();
  }
}

int main(int argc, char** argv)
{
  using pcl::PointCloud;
  using pcl::PointXYZ;

  PointCloud<PointXYZ> input_cloud;
  for (int i = 0; i < 10; i++)
  {
    double angle = i / 10.0 * M_PI;
    input_cloud.points.emplace_back(cos(angle), sin(angle), 0.0);
  }

  PointCloud<PointXYZ> target_cloud;
  for (int i = 0; i < 10; i++)
  {
    double angle = (i / 10.0 + 0.04) * M_PI;
    target_cloud.points.emplace_back(cos(angle), sin(angle), 0.0);
  }

  scanmatcher::ICP icp;
  auto [transform, error] = icp.scanmatch(input_cloud, target_cloud);
  pcl::PointCloud transformed{input_cloud};
  transformPointCloud(transformed, transform);

  std::cout << "Returned! " << std::endl;
  std::cout << "translation: " << transform.translation() << std::endl;
  std::cout << "rotation: " << Eigen::AngleAxisd(transform.rotation()).angle() << std::endl;
  std::cout << "error: " << error << std::endl;

  ros::init(argc, argv, "week_scanmatching");

  ros::NodeHandle nh;

  ros::Publisher original_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("original", 1, true);
  ros::Publisher target_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("target", 1, true);
  ros::Publisher transformed_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed", 1, true);
  ros::Duration(2).sleep();

  input_cloud.header.frame_id = "odom";
  input_cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  original_pub.publish(input_cloud);

  target_cloud.header.frame_id = "odom";
  target_cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  target_pub.publish(target_cloud);

  transformed.header.frame_id = "odom";
  transformed.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  transformed_pub.publish(transformed);

  ros::spin();
  std::exit(0);
}
