cmake_minimum_required(VERSION 2.8.3)
project(softfoot_thing_visualization)

add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  tf
  tf_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
  # LIBRARIES ${PROJECT_NAME}_lib
)

set(OBJECTS_SOURCE_FILES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

# NODE: Lower level position controls
# add_executable(control_server src/control_server.cpp ${OBJECTS_SOURCE_FILES})
# add_dependencies(control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(control_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})