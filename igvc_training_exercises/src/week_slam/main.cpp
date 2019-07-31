#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week_slam");

  ros::NodeHandle nh;

  ros::spin();
}
