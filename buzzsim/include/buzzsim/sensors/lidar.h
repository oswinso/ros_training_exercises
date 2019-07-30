#ifndef SRC_LIDAR_H
#define SRC_LIDAR_H

#include <vector>

#include <pcl_ros/point_cloud.h>

#include <buzzsim/motion.h>
#include <buzzsim/obstacle.h>
#include <buzzsim/sensors/cgal_types.h>

namespace turtle
{
class Lidar
{
public:
  struct Options
  {
    double angle_width;
    double range;
    double angular_resolution;
  };

  Lidar(const Options& options);
  pcl::PointCloud<pcl::PointXY> getLidarScan(const motion::Pose& pose, const std::vector<Obstacle>* obstacles);
  void test();
  void test2(const motion::Position& position, const std::vector<Obstacle>& obstacles);
  Polygon_2 computeVisibilityPolygon(const motion::Position& position, const Polygon_with_holes_2& polygon) const;
  Polygon_with_holes_2 getPolygonWithHoles(const motion::Position& position, const std::vector<Obstacle>& obstacles) const;

private:
  struct LidarHit
  {
    double angle;
    double range;
  };

  std::vector<LidarHit> getLidarScan(const motion::Pose& pose, const Obstacle& obstacle);

  Options options_;
};
}  // namespace turtle

#endif  // SRC_LIDAR_H
