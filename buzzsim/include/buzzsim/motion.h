#ifndef SRC_GEOMETRY_H
#define SRC_GEOMETRY_H

#include <QPointF>
#include <ostream>
#include <geometry_msgs/Pose.h>

namespace motion
{
constexpr double SECS_PER_UPDATE = 0.016;
constexpr double PIXELS_PER_M = 50.0;

struct Twist;
struct TwistLimits;
struct Acceleration;
struct AccelerationLimits;

struct Position
{
  double x{};
  double y{};

  QPointF toQPointF(int width, int height) const;
};

struct AccelerationLimits
{
  double linear = std::numeric_limits<double>::max();
  double angular = std::numeric_limits<double>::max();

  Acceleration limit(const Acceleration& acceleration) const;
};

struct Acceleration
{
  double linear{};
  double angular{};
};

struct TwistLimits
{
  double linear = std::numeric_limits<double>::max();
  double angular = std::numeric_limits<double>::max();

  Twist limit(const Twist& twist) const;
};

struct Twist
{
  double linear{};
  double angular{};

  void apply(const Acceleration& acceleration, const TwistLimits& limits = {});
  Twist getApplied(const Acceleration& acceleration, const TwistLimits& limits = {}) const;

  Twist operator+(const Twist& other) const;
  Twist operator/(double scalar) const;
  bool operator==(const Twist& other) const;
};

struct Limits
{
  AccelerationLimits acceleration{};
  TwistLimits twist{};
};

struct Pose
{
  Position position{};
  double orientation{};

  void apply(const Twist& twist);
  Pose getApplied(const Twist& twist) const;
  geometry_msgs::Pose toROSMsg() const;

  void normalizeHeading();
};

struct State
{
  Pose pose;
  Twist twist;

  explicit State(Pose pose = {}, Twist twist = {});
  void apply(const Acceleration& acceleration, const Limits& limits = {});
};

std::ostream& operator<<(std::ostream& out, const Position& twist);
std::ostream& operator<<(std::ostream& out, const Pose& twist);
std::ostream& operator<<(std::ostream& out, const Twist& twist);
std::ostream& operator<<(std::ostream& out, const Acceleration& twist);
std::ostream& operator<<(std::ostream& out, const State& twist);
}  // namespace motion

#endif  // SRC_GEOMETRY_H
