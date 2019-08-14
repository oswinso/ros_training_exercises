#ifndef SRC_EDGE_LANDMARK_H
#define SRC_EDGE_LANDMARK_H

#include <g2o/core/base_binary_edge.h>
#include <week_slam/g2o/vertex/vertex_robot_state.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <week_slam/g2o/landmark_measurement.h>

namespace g2o
{
class EdgeLandmark : public BaseBinaryEdge<2, LandmarkMeasurement, VertexRobotState, VertexPointXY>
{
 public:
  EdgeLandmark() = default;
  void computeError() override;
  void setMeasurement(const LandmarkMeasurement& m) override;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

 private:
  double delta_v_{};
};
}  // namespace g2o

#endif //SRC_EDGE_LANDMARK_H
