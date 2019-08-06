#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publisher");
  std::cout << "Hello World!" << std::endl;

  ros::NodeHandle nh;
  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);

  ros::Duration(1).sleep();
  std_msgs::Int32 message;
  message.data = 13;
  integer_pub.publish(message);

  ros::spin();
}
