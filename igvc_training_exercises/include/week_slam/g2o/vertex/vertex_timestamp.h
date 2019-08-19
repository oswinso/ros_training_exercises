#ifndef SRC_VERTEX_TIMESTAMP_H
#define SRC_VERTEX_TIMESTAMP_H

#include <ros/time.h>
#include <g2o/core/base_vertex.h>
#include <week_slam/g2o/twist.h>

namespace g2o
{
class VertexTimestamp : public g2o::BaseVertex<1, ros::Time>
{
public:
  VertexTimestamp() = default;

  void setToOriginImpl() override
  {
    _estimate = ros::Time::now();
  }

  bool read(std::istream &is) override
  {
    is >> _estimate.sec >> _estimate.nsec;
    return true;
  }

  bool write(std::ostream &os) const override
  {
    os << _estimate.sec << " " << _estimate.nsec;
    return os.good();
  }

  void oplusImpl(const number_t *update) override
  {
    _estimate += ros::Duration(update[0]);
  }
};
}  // namespace g2o

#endif  // SRC_VERTEX_TIMESTAMP_H
