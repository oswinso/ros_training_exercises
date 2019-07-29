#include <buzzsim/sensors/lidar.h>

namespace turtle
{
Lidar::Lidar(const LidarOptions& options) : options_{ options }
{
}

pcl::PointCloud<pcl::PointXYZ> Lidar::getLidarScan(const motion::Pose& pose, const std::vector<Obstacle>* obstacles)
{
  // Idea:
  // Sort each vertex by angle
  // Iterate through vertices in order
  // If distance is closer, then go calculate
  // If encounter end of segment, then look at what's currently on the stack?

  // Heap of segments (sorted by distance)
  // Heap of (vertices, start / end bool)
  // bimap of start vertex <-> end vertex?

  // Iterate through heap
  // if start vertex, add to b tree
  // if end vertex of current segment, then calculate all points and add them in
  // if end vertex of random segment, then remove that random one's start vertex,
  // if tree is empty, then keep going
  // if smaller than, then still add but don't add point
  // if larger than max angle, then terminate.
}

std::vector<Lidar::LidarHit> Lidar::getLidarScan(const motion::Pose& pose, const Obstacle& obstacle)
{
  std::vector<LidarHit> hits;
}
}  // namespace turtle
