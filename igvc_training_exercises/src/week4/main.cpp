#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

struct Position
{
  double x;
  double y;
};

struct Pose
{
  Position position;
  double heading;
};

struct Twist
{
  double linear;
  double angular;
};

struct State
{
  Pose pose;
  Twist twist;
};

State g_state;
ros::Time g_last_time;
ros::Publisher g_odom_pub;

void imuCallback(const sensor_msgs::Imu& msg)
{
//  g_state.pose.heading = tf::getYaw(msg.orientation);
  g_state.twist.angular = msg.angular_velocity.z;

  ros::Duration delta_t = msg.header.stamp - g_last_time;
  g_state.twist.linear += msg.linear_acceleration.x * delta_t.toSec();
  g_state.pose.heading += g_state.twist.angular * delta_t.toSec();

  g_state.pose.position.x += cos(g_state.pose.heading) * g_state.twist.linear * delta_t.toSec();
  g_state.pose.position.y += sin(g_state.pose.heading) * g_state.twist.linear * delta_t.toSec();

  g_last_time = msg.header.stamp;

  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.frame_id = "odom";
  odometry_msg.header.stamp = msg.header.stamp;
  odometry_msg.pose.pose.position.x = g_state.pose.position.x;
  odometry_msg.pose.pose.position.y = g_state.pose.position.y;
  odometry_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_state.pose.heading);
  odometry_msg.twist.twist.linear.x = g_state.twist.linear;
  odometry_msg.twist.twist.angular.z = g_state.twist.angular;
  g_odom_pub.publish(odometry_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week_4");

  ros::NodeHandle nh;
  ros::NodeHandle pnh{"~"};

  ros::Subscriber imu_sub = nh.subscribe("oswin/imu", 1, imuCallback);
  g_odom_pub = nh.advertise<nav_msgs::Odometry>("oswin/odometry", 1);

  g_last_time = ros::Time::now();
  ros::spin();
}
