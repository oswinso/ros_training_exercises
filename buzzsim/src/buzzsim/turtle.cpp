#include <utility>

#include <utility>

#include <cmath>

#include <QPainter>

#include <buzzsim/turtle.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

namespace turtle
{
Turtle::Turtle(const Options& options)
  : state_{ options.state }
  , limits_{ options.limits }
  , setpoint_{ options.state.twist }
  , name_{ options.name }
  , turtle_image_{ options.turtle_image }
{
  ROS_INFO_STREAM("Created turtle " << name_);
  updateRotatedImage();

  velocity_sub_ = nh_.subscribe(name_ + "/velocity", 1, &Turtle::velocityCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "/ground_truth", 1);
  publish_timer_ = nh_.createTimer(ros::Duration(0.1), &Turtle::publishCallback, this);
}

void Turtle::paint(QPainter& painter)
{
  auto width = painter.window().width();
  auto height = painter.window().height();
  QPointF p = state_.pose.position.toQPointF(width, height);
  p.rx() -= 0.5 * turtle_rotated_image_.width();
  p.ry() -= 0.5 * turtle_rotated_image_.height();
  painter.drawImage(p, turtle_rotated_image_);
}

void Turtle::updateRotatedImage()
{
  QTransform transform;
  transform.rotate(-state_.pose.orientation * 180.0 / M_PI);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
}

void Turtle::updatePose()
{
  double last_orientation = state_.pose.orientation;
  auto acceleration = getAcceleration();

  state_.apply(acceleration, limits_);

  if (state_.pose.orientation != last_orientation)
  {
    updateRotatedImage();
  }
}

void Turtle::velocityCallback(const geometry_msgs::TwistConstPtr& twist)
{
  std::unique_lock<std::mutex> lock(setpoint_mutex_);
  setpoint_.linear = twist->linear.x;
  setpoint_.angular = twist->angular.z;
}

void Turtle::publishCallback(const ros::TimerEvent& event)
{
  geometry_msgs::PoseStamped msg;
  msg.pose = state_.pose.toROSMsg();
  msg.header.frame_id = name_;
  msg.header.stamp = ros::Time::now();
  pose_pub_.publish(msg);
}

motion::Acceleration Turtle::getAcceleration()
{
  std::unique_lock<std::mutex> lock(setpoint_mutex_);
  auto d_x = setpoint_.linear - state_.twist.linear;
  auto d_theta = setpoint_.angular - state_.twist.angular;

  return { d_x / motion::SECS_PER_UPDATE, d_theta / motion::SECS_PER_UPDATE };
}

std::ostream& operator<<(std::ostream& os, const Turtle::Options& options)
{
  os << "Turtle:" << std::endl;
  os << "\t"
     << "name: " << options.name << std::endl;
  os << "\t"
     << "state: " << options.state << std::endl;
  return os;
}
}  // namespace turtle
