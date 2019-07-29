add_executable(week3 main.cpp)
target_include_directories(week3 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week3 ${catkin_LIBRARIES})
add_dependencies(week3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week3_world_setup world_setup.cpp)
target_include_directories(week3_world_setup PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week3_world_setup ${catkin_LIBRARIES})
add_dependencies(week3_world_setup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
