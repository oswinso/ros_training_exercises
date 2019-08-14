#ifndef SRC_LANDMARK_MEASUREMENT_H
#define SRC_LANDMARK_MEASUREMENT_H

#include <week_slam/g2o/g2o_robot_state.h>

namespace g2o
{
class LandmarkMeasurement
{
public:
  LandmarkMeasurement() = default;

  LandmarkMeasurement(int id, double x, double y);

  [[nodiscard]] const int& id() const
  {
    return id_;
  }

  [[nodiscard]] const double& x() const
  {
    return x_;
  }

  [[nodiscard]] double& x()
  {
    return x_;
  }

  [[nodiscard]] const double& y() const
  {
    return y_;
  }

  [[nodiscard]] double& y()
  {
    return y_;
  }

  void fromVector(const Eigen::Vector3d& v)
  {
    *this = LandmarkMeasurement{ static_cast<int>(v[0]), v[1], v[2] };
  }

  [[nodiscard]] Eigen::Vector3d toVector() const
  {
    return { static_cast<double>(id_), x_, y_ };
  }

private:
  int id_;
  double x_;
  double y_;
};
}  // namespace g2o

#endif  // SRC_LANDMARK_MEASUREMENT_H
