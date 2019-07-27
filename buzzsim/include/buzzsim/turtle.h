#ifndef SRC_TURTLE_H
#define SRC_TURTLE_H

#include <mutex>

#include <QImage>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>

#include <buzzsim/motion.h>

namespace turtle
{
class Turtle
{
public:
  struct Options
  {
    std::string name;
    QImage turtle_image;
    motion::State state{};
    motion::Limits limits{};
  };

  Turtle(const Options& options);
  void paint(QPainter &painter);
  void updatePose();

  motion::State state_;
  motion::Limits limits_;
  motion::Twist setpoint_{};

private:
  void updateRotatedImage();
  void velocityCallback(const geometry_msgs::TwistConstPtr& twist);
  void publishCallback(const ros::TimerEvent& event);
  motion::Acceleration getAcceleration();

  std::string name_;

  std::mutex setpoint_mutex_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  ros::NodeHandle nh_;
  ros::Subscriber velocity_sub_;
  ros::Publisher pose_pub_;
  ros::Timer publish_timer_;
};

std::ostream& operator<<(std::ostream& os, const Turtle::Options& options);
}  // namespace turtle
#endif  // SRC_TURTLE_H
