add_executable(week_slam main.cpp)
  target_include_directories(week_slam PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(week_slam ${catkin_LIBRARIES})
add_dependencies(week_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
