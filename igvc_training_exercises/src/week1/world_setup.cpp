#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "world_setup");

  ros::NodeHandle nh;

  ros::Publisher kyle_velocity_pub = nh.advertise<geometry_msgs::Twist>("kyle/velocity", 1);

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 1.0;
  while (ros::ok())
  {
    kyle_velocity_pub.publish(twist_msg);
  }
}
