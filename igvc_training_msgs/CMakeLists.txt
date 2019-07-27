cmake_minimum_required(VERSION 2.8.3)
project(igvc_training_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
             message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             )

# Generate messages in the 'msg' folder
#add_message_files(
#    DIRECTORY msg
#    FILES
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#)

catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
)

