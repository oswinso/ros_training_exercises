#ifndef SRC_EDGE_IMU_H
#define SRC_EDGE_IMU_H

#include <g2o/core/base_multi_edge.h>
#include <week_slam/g2o/IMU_measurement.h>

namespace g2o
{
class EdgeIMU : public BaseMultiEdge<2, IMUMeasurement>
{
public:
  EdgeIMU();
  void computeError() override;
  void setMeasurement(const IMUMeasurement& m) override;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

private:
  double delta_v_{};
};
}  // namespace g2o

#endif  // SRC_EDGE_IMU_H
