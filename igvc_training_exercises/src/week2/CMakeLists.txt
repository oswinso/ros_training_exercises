add_executable(week2 main.cpp)
target_include_directories(week2 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week2 ${catkin_LIBRARIES})
add_dependencies(week2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
