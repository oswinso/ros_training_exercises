add_executable(week4 main.cpp)
target_include_directories(week4 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week4 ${catkin_LIBRARIES})
add_dependencies(week4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week4_world_setup world_setup.cpp)
target_include_directories(week4_world_setup PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week4_world_setup ${catkin_LIBRARIES})
add_dependencies(week4_world_setup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
