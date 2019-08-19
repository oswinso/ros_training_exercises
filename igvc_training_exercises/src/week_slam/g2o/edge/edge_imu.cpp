#include <g2o/types/slam2d/vertex_se2.h>

#include <week_slam/g2o/edge/edge_imu.h>
#include <week_slam/g2o/vertex/vertex_twist.h>
#include <week_slam/g2o/vertex/vertex_timestamp.h>

namespace g2o
{
EdgeIMU::EdgeIMU() : BaseMultiEdge<2, IMUMeasurement>()
{
  resize(4);
}

void EdgeIMU::computeError()
{
  // TODO(Oswin): Remove time information from IMU measurement
  const auto *from_twist = static_cast<const VertexTwist *>(_vertices[0]);
  const auto *from_stamp = static_cast<const VertexTimestamp *>(_vertices[1]);
  const auto *to_twist = static_cast<const VertexTwist *>(_vertices[2]);
  const auto *to_stamp = static_cast<const VertexTimestamp *>(_vertices[3]);

  _error(0) = (to_twist->estimate().linear() - from_twist->estimate().linear()) - delta_v_;
  _error(1) = (from_twist->estimate().angular() + to_twist->estimate().angular()) / 2 - _measurement.angular_velocity();
}

void EdgeIMU::setMeasurement(const IMUMeasurement &m)
{
  _measurement = m;
}

bool EdgeIMU::read(std::istream &is)
{
  Eigen::Vector3d p;
  is >> p[0] >> p[1] >> p[2];
  _measurement.fromVector(p);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  }

  return true;
}

bool EdgeIMU::write(std::ostream &os) const
{
  Eigen::Vector3d p = measurement().toVector();
  os << p.x() << " " << p.y() << " " << p.z();

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      os << " " << information()(i, j);
    }
  }

  return os.good();
}
}  // namespace g2o
