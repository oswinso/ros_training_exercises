#include <cmath>

#include <QPainter>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <buzzsim/turtle.h>

namespace turtle
{
Turtle::Turtle(const Options& options)
  : state_{ options.state }
  , limits_{ options.limits }
  , setpoint_{ options.state.twist }
  , publish_options_{ options.publish_options }
  , name_{ options.name }
  , turtle_image_{ options.turtle_image }
  , imu_noise_{ options.sensor_std_devs }
{
  ROS_INFO_STREAM("Created turtle " << name_);
  updateRotatedImage();

  velocity_sub_ = nh_.subscribe(name_ + "/velocity", 1, &Turtle::velocityCallback, this);

  if (publish_options_.imu)
  {
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(name_ + "/imu", 1);
  }

  if (publish_options_.pose)
  {
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "/ground_truth", 1);
  }

  if (publish_options_.hasPublisher())
  {
    publish_timer_ = nh_.createTimer(ros::Duration(0.1), &Turtle::publishCallback, this);
  }
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
  acceleration_ = getAcceleration();

  state_.apply(acceleration_, limits_);

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

void Turtle::publishCallback([[maybe_unused]] const ros::TimerEvent&)
{
  if (publish_options_.pose)
  {
    publishPose();
  }

  if (publish_options_.imu)
  {
    publishIMU();
  }
}

void Turtle::publishPose()
{
  geometry_msgs::PoseStamped msg;
  msg.pose = state_.pose.toROSMsg();
  msg.header.frame_id = name_;
  msg.header.stamp = ros::Time::now();
  pose_pub_.publish(msg);
}

void Turtle::publishIMU()
{
  sensor_msgs::Imu msg{};
  msg.header.frame_id = name_;
  msg.header.stamp = ros::Time::now();
  msg.angular_velocity.z = state_.twist.angular + imu_noise_.gyroNoise();
  msg.linear_acceleration.x = acceleration_.linear + imu_noise_.accelerometerNoise();
  msg.orientation = tf::createQuaternionMsgFromYaw(state_.pose.orientation + imu_noise_.magnetometerNoise());

  msg.linear_acceleration_covariance[0] = imu_noise_.sensor_std_devs_.accelerometer;
  msg.angular_velocity_covariance[0] = imu_noise_.sensor_std_devs_.gyro;
  msg.orientation_covariance[0] = imu_noise_.sensor_std_devs_.magnetometer;

  imu_pub_.publish(msg);
}

motion::Acceleration Turtle::getAcceleration()
{
  std::unique_lock<std::mutex> lock(setpoint_mutex_);
  auto d_x = setpoint_.linear - state_.twist.linear;
  auto d_theta = setpoint_.angular - state_.twist.angular;

  return { d_x / motion::SECS_PER_UPDATE, d_theta / motion::SECS_PER_UPDATE };
}

bool Turtle::PublishOptions::hasPublisher() const
{
  return imu || pose;
}

Turtle::IMUNoise::IMUNoise(const SensorStdDevs& sensor_std_devs)
  : sensor_std_devs_{ sensor_std_devs }
  , acccelerometer_noise_{ 0, sensor_std_devs.accelerometer }
  , gyro_noise_{ 0, sensor_std_devs.gyro }
  , magnetometer_noise_{ sensor_std_devs.magnetometer }
{
}

double Turtle::IMUNoise::accelerometerNoise()
{
  return acccelerometer_noise_(gen_);
}

double Turtle::IMUNoise::gyroNoise()
{
  return gyro_noise_(gen_);
}

double Turtle::IMUNoise::magnetometerNoise()
{
  return magnetometer_noise_(gen_);
}

std::ostream& operator<<(std::ostream& os, const Turtle::Options& options)
{
  os << "Turtle:" << std::endl;
  os << "\t"
     << "name: " << options.name << std::endl;
  os << "\t"
     << "state: " << options.state << std::endl;
  os << "\t"
     << "publishers: " << (options.publish_options.imu ? "imu " : "") << (options.publish_options.pose ? "pose " : "")
     << std::endl;
  os << "\t"
     << "std_devs: " << std::endl;
  os << "\t\t"
     << "accelerometer: " << options.sensor_std_devs.accelerometer << std::endl;
  os << "\t\t"
     << "gyro: " << options.sensor_std_devs.gyro << std::endl;
  os << "\t\t"
     << "magnetometer: " << options.sensor_std_devs.magnetometer << std::endl;
  return os;
}
}  // namespace turtle
