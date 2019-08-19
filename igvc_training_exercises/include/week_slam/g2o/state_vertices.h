#ifndef SRC_STATE_VERTICES_H
#define SRC_STATE_VERTICES_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <week_slam/g2o/vertex/vertex_timestamp.h>
#include <week_slam/g2o/vertex/vertex_twist.h>
#include <week_slam/g2o/edge/edge_imu.h>
#include <week_slam/g2o/edge/edge_holonomic.h>

namespace g2o
{
struct StateVertices
{
  StateVertices() = default;
  StateVertices(int id, SparseOptimizer& optimizer)
    : se2(static_cast<VertexSE2*>(optimizer.vertex(id)))
    , twist(static_cast<VertexTwist*>(optimizer.vertex(id + 1)))
    , stamp(static_cast<VertexTimestamp*>(optimizer.vertex(id + 2)))
  {
  }

  void allocate()
  {
    assert(se2 == nullptr);
    assert(twist == nullptr);
    assert(stamp == nullptr);

    se2 = new VertexSE2();
    twist = new VertexTwist();
    stamp = new VertexTimestamp();
  }

  void setIdsFrom(int starting_id) const
  {
    se2->setId(starting_id);
    twist->setId(starting_id + 1);
    stamp->setId(starting_id + 2);
  }

  void insertIntoGraph(SparseOptimizer& optimizer) const
  {
    optimizer.addVertex(se2);
    optimizer.addVertex(twist);
    optimizer.addVertex(stamp);
  }

  void connectIMUEdge(const StateVertices& next, EdgeIMU& edge) const
  {
    edge.setVertex(0, twist);
    edge.setVertex(1, stamp);
    edge.setVertex(2, next.twist);
    edge.setVertex(3, next.stamp);
  }

  void connectHolonomicEdge(const StateVertices& next, EdgeHolonomic& edge) const
  {
    edge.setVertex(0, se2);
    edge.setVertex(1, twist);
    edge.setVertex(3, stamp);
    edge.setVertex(4, next.se2);
    edge.setVertex(5, next.twist);
    edge.setVertex(6, next.stamp);
  }

  VertexSE2* se2 = nullptr;
  VertexTwist* twist = nullptr;
  VertexTimestamp* stamp = nullptr;
};
}  // namespace g2o

#endif  // SRC_STATE_VERTICES_H
