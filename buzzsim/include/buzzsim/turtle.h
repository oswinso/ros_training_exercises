#ifndef SRC_TURTLE_H
#define SRC_TURTLE_H

#include <mutex>
#include <random>

#include <QImage>

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <buzzsim/motion.h>

namespace turtle
{
class Turtle
{
public:
  struct PublishOptions
  {
    bool imu;
    bool pose;

    [[nodiscard]] bool hasPublisher() const;
  };

  struct SensorStdDevs
  {
    double accelerometer{};
    double gyro{};
    double magnetometer{};
  };

  struct Options
  {
    std::string name;
    QImage turtle_image;
    motion::State state{};
    motion::Limits limits{};
    SensorStdDevs sensor_std_devs{};
    PublishOptions publish_options{};
  };

  struct IMUNoise
  {
    std::mt19937 gen_{ std::random_device{}() };
    SensorStdDevs sensor_std_devs_{};
    std::normal_distribution<> acccelerometer_noise_{};
    std::normal_distribution<> gyro_noise_{};
    std::normal_distribution<> magnetometer_noise_{};

    IMUNoise(const SensorStdDevs& sensor_std_devs);
    double accelerometerNoise();
    double gyroNoise();
    double magnetometerNoise();
  };

  Turtle(const Options& options);
  void paint(QPainter& painter);
  void updatePose();

  motion::State state_;
  motion::Acceleration acceleration_;
  motion::Limits limits_;
  motion::Twist setpoint_{};

  PublishOptions publish_options_{};

private:
  void updateRotatedImage();
  void velocityCallback(const geometry_msgs::TwistConstPtr& twist);
  void publishCallback([[maybe_unused]] const ros::TimerEvent&);

  void publishPose();
  void publishIMU();

  motion::Acceleration getAcceleration();

  std::string name_;

  std::mutex setpoint_mutex_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  ros::NodeHandle nh_;
  ros::Subscriber velocity_sub_;

  ros::Publisher imu_pub_;
  ros::Publisher pose_pub_;

  ros::Timer publish_timer_;

  IMUNoise imu_noise_;
};

std::ostream& operator<<(std::ostream& os, const Turtle::Options& options);
}  // namespace turtle
#endif  // SRC_TURTLE_H
