#ifndef SRC_EDGE_HOLONOMIC_H
#define SRC_EDGE_HOLONOMIC_H

#include <ros/ros.h>
#include <g2o/core/base_multi_edge.h>
#include <week_slam/g2o/IMU_measurement.h>

namespace g2o
{
class EdgeHolonomic : public BaseMultiEdge<0, int>
{
public:
  EdgeHolonomic();

public:
  void computeError() override;

  bool read(std::istream& is) override
  {
    return true;
  }

  bool write(std::ostream& os) const override
  {
    os << "holonomic edge";
    return os.good();
  }
};
}  // namespace g2o

#endif  // SRC_EDGE_HOLONOMIC_H
