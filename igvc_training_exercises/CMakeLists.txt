cmake_minimum_required(VERSION 3.9.0)
project(igvc_training_exercises)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-W -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  igvc_buzzsim)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  roscpp
  tf
  igvc_buzzsim)

include_directories(include)

add_subdirectory(src/week3)
add_subdirectory(src/week4)
add_subdirectory(src/week5)
add_subdirectory(src/week6)
