find_package(Eigen3 3.3 REQUIRED)
add_library(scanmatching icp/icp.cpp)
target_include_directories(scanmatching PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(scanmatching ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(scanmatching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week_scanmatching
  main.cpp
  scanmatching_localizer.cpp)
target_include_directories(week_scanmatching PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week_scanmatching ${catkin_LIBRARIES} scanmatching)
add_dependencies(week_scanmatching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} scanmatching)
