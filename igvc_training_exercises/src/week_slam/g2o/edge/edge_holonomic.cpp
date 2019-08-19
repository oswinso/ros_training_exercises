#include <g2o/types/slam2d/vertex_se2.h>
#include <week_slam/g2o/vertex/vertex_timestamp.h>
#include <week_slam/g2o/vertex/vertex_twist.h>

#include <week_slam/g2o/edge/edge_holonomic.h>

namespace g2o
{
EdgeHolonomic::EdgeHolonomic() : BaseMultiEdge<0, int>()
{
  resize(6);
}

void EdgeHolonomic::computeError()
{
  const auto* from_se2 = static_cast<const VertexSE2*>(_vertices[0]);
  const auto* from_twist = static_cast<const VertexTwist*>(_vertices[1]);
  const auto* from_stamp = static_cast<const VertexTimestamp*>(_vertices[2]);
  const auto* to_se2 = static_cast<const VertexSE2*>(_vertices[3]);
  const auto* to_twist = static_cast<const VertexTwist*>(_vertices[4]);
  const auto* to_stamp = static_cast<const VertexTimestamp*>(_vertices[5]);

  Twist average_twist = (from_twist->estimate() + to_twist->estimate()) / 2.0;

  ros::Duration delta_t = to_stamp->estimate() - from_stamp->estimate();

  SE2 predicted_se2 = average_twist.apply(from_se2->estimate(), delta_t.toSec());

  _error = (predicted_se2.inverse() * to_se2->estimate()).toVector();
}
}  // namespace g2o
