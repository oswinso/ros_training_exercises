#ifndef SRC_LIDAR_H
#define SRC_LIDAR_H

#include <vector>

#include <pcl_ros/point_cloud.h>

#include <buzzsim/motion.h>
#include <buzzsim/obstacle.h>

namespace turtle
{
class Lidar
{
public:
  struct LidarOptions
  {
    double angle_width;
    double range;
    double angular_resolution;
  };

  Lidar(const LidarOptions& options);
  pcl::PointCloud<pcl::PointXYZ> getLidarScan(const motion::Pose& pose, const std::vector<Obstacle>* obstacles);

private:
  struct LidarHit
  {
    double angle;
    double range;
  };

  std::vector<LidarHit> getLidarScan(const motion::Pose& pose, const Obstacle& obstacle);

  LidarOptions options_;
};
}  // namespace turtle

#endif  // SRC_LIDAR_H
