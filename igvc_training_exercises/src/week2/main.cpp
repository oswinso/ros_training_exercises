#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  std::cout << "Hello World!" << std::endl;

  ros::spin();
}
