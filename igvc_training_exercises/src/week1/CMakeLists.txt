add_executable(week1 main.cpp)
target_include_directories(week1 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week1 ${catkin_LIBRARIES})
add_dependencies(week1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(world_setup world_setup.cpp)
target_include_directories(world_setup PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(world_setup ${catkin_LIBRARIES})
add_dependencies(world_setup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
