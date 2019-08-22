#include <week_slam/landmark_registration.h>
#include <pcl_ros/transforms.h>

LandmarkRegistration::LandmarkRegistration(const Options& options, const BarrelRansac::Options& barrel_ransac_options)
  : options_{ options }, barrel_ransac_{ barrel_ransac_options }
{
}

std::vector<Landmark> LandmarkRegistration::getLandmarks(const pcl::PointCloud<pcl::PointXYZ>& scan,
                                                         const tf::Transform& transform_to_odom)
{
  std::vector<Barrel> result = barrel_ransac_.execute(scan);

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  if (result.size() > 3 && landmarks_.size() > 3)
  {
    pcl::PointCloud<pcl::PointXYZ> ransac_result_cloud = toPointCloud(result);
    pcl_ros::transformPointCloud(ransac_result_cloud, ransac_result_cloud, transform_to_odom);
    pcl::PointCloud<pcl::PointXYZ> landmarks_cloud = toPointCloud(landmarks_);
    auto scanmatch_result = icp_.scanmatch(ransac_result_cloud, landmarks_cloud);

    if (scanmatch_result.error < 1.0)
    {
      transform = scanmatch_result.transform;
    }
//    ROS_INFO_STREAM("\nError: " << scanmatch_result.error << "\nTransform: rotation:\n" << transform.rotation() << "\ntranslation:\n" << transform.translation());
  }

  std::vector<Landmark> landmarks;
  landmarks.reserve(result.size());
  for (const auto& barrel : result)
  {
    auto scanmatched_barrel = transform * barrel;
    auto transformed_barrel_tf = transform_to_odom * tf::Vector3{ scanmatched_barrel(0), scanmatched_barrel(1), 0.0 };
    auto transformed_barrel = Eigen::Vector3d{ transformed_barrel_tf.x(), transformed_barrel_tf.y(), barrel(2) };

    int closest_index = findClosestLandmark(transformed_barrel);
    if (closest_index == -1)
    {
      closest_index = registerLandmark(transformed_barrel);
    }
    Landmark closest_landmark{ closest_index, barrel };
    landmarks.emplace_back(closest_landmark);
  }

  return landmarks;
}

int LandmarkRegistration::findClosestLandmark(const Barrel& barrel) const
{
  double min_distance = std::numeric_limits<double>::max();
  int min_index = 0;
  for (int i = 0; i < static_cast<int>(landmarks_.size()); i++)
  {
    double distance = (landmarks_[i].barrel.head<2>() - barrel.head<2>()).norm();
    if (distance < min_distance)
    {
      min_distance = distance;
      min_index = i;
    }
  }

  if (min_distance < options_.new_landmark_threshold)
  {
    return min_index;
  }
  ROS_INFO_STREAM("distance: " << min_distance << ", creating new landmark with id " << landmarks_.size());

  return -1;
}

int LandmarkRegistration::registerLandmark(const Barrel& barrel)
{
  int id = landmarks_.size();
  Landmark new_landmark{ id, barrel };
  landmarks_.emplace_back(new_landmark);
  return id;
}

void LandmarkRegistration::updateLandmark(int id, const Eigen::Vector2d& location)
{
  landmarks_.at(id).barrel.head<2>() = location;
}

pcl::PointCloud<pcl::PointXYZ> LandmarkRegistration::toPointCloud(const std::vector<Barrel>& barrels) const
{
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  for (const auto& barrel : barrels)
  {
    pointcloud.points.emplace_back(barrel(0), barrel(1), 0.0);
  }
  return pointcloud;
}

pcl::PointCloud<pcl::PointXYZ> LandmarkRegistration::toPointCloud(const std::vector<Landmark>& landmarks) const
{
  const auto toBarrel = [](const Landmark& landmark) { return landmark.barrel; };
  std::vector<Barrel> barrels(landmarks.size());
  std::transform(landmarks.begin(), landmarks.end(), std::back_inserter(barrels), toBarrel);

  return toPointCloud(barrels);
}
