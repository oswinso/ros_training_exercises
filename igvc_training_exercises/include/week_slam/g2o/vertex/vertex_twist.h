#ifndef SRC_VERTEX_TWIST_H
#define SRC_VERTEX_TWIST_H

#include <g2o/core/base_vertex.h>
#include <week_slam/g2o/twist.h>

namespace g2o
{
class VertexTwist : public g2o::BaseVertex<2, Twist>
{
 public:
  VertexTwist() = default;

  void setToOriginImpl() override;
  bool read(std::istream &is) override;
  bool write(std::ostream &os) const override;
  void oplusImpl(const number_t *update) override;
};
}  // namespace g2o

#endif //SRC_VERTEX_TWIST_H
