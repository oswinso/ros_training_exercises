cmake_minimum_required(VERSION 3.9.0)
project(igvc_buzzsim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-W -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  std_msgs
  turtlesim
  std_srvs)

###########################
#       QT stuff          #
###########################
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  roslib
  geometry_msgs
  message_runtime
  std_msgs
  std_srvs)

###########
## Build ##
###########
set(buzzsim_SRCS
  src/buzzsim/main.cpp
  src/buzzsim/app.cpp
  src/buzzsim/frame.cpp
  src/buzzsim/turtle.cpp
  src/buzzsim/motion.cpp
  src/buzzsim/world.cpp
  )
set(buzzsim_HDRS
  include/buzzsim/app.h
  include/buzzsim/frame.h
  include/buzzsim/turtle.h
  )

qt5_wrap_cpp(buzzsim_MOCS ${buzzsim_HDRS})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(buzzsim_node ${buzzsim_SRCS} ${buzzsim_MOCS})
target_include_directories(buzzsim_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(buzzsim_node ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(buzzsim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
