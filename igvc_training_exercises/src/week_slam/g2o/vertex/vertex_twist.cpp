#include <week_slam/g2o/vertex/vertex_twist.h>
namespace g2o
{
void VertexTwist::setToOriginImpl()
{
  _estimate = Twist{};
}

bool VertexTwist::read(std::istream &is)
{
  Eigen::Vector2d p;
  is >> p(0) >> p(1);
  _estimate.fromVector(p);

  return true;
}

bool VertexTwist::write(std::ostream &os) const
{
  Eigen::Vector2d p = estimate().toVector();
  os << p(0) << " " << p(1);

  return os.good();
}

void VertexTwist::oplusImpl(const number_t *update)
{
  _estimate.linear() += update[0];
  _estimate.angular() += update[1];
}
}  // namespace g2o
