#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher g_velocity_pub;
ros::Publisher g_error_pub;

geometry_msgs::PoseStamped g_kyle_pose;
ros::Time g_last_time;

double g_last_error;
double g_accumulator;

void kylePoseCallback(const geometry_msgs::PoseStamped& msg)
{
  g_kyle_pose = msg;
}

void oswinPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  double error = g_kyle_pose.pose.position.x - msg.pose.position.x;
  ros::Duration delta_t = msg.header.stamp - g_last_time;

  double kp = 0.5;
  double ki = 0.0;
  double kd = 0.0;

  double proportional = error;
  double derivative = (error - g_last_error) / delta_t.toSec();
  double integral = g_accumulator;

  double control = kp * proportional + kd * derivative + ki * integral;

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = control;
  g_velocity_pub.publish(twist_msg);

  std_msgs::Float64 error_msg;
  error_msg.data = error;
  g_error_pub.publish(error_msg);

  g_accumulator += error * delta_t.toSec();
  g_last_error = error;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week1");

  ros::NodeHandle nh;

  g_velocity_pub = nh.advertise<geometry_msgs::Twist>("oswin/velocity", 1);
  g_error_pub = nh.advertise<std_msgs::Float64>("error", 1);
  ros::Subscriber kyle_sub = nh.subscribe("kyle/ground_truth", 1, kylePoseCallback);
  ros::Subscriber oswin_sub = nh.subscribe("oswin/ground_truth", 1, oswinPoseCallback);
  ros::spin();
}
