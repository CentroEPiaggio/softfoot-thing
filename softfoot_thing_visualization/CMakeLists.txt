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
  urdf
  kdl_parser
  qb_interface
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs message_runtime
)

set(OBJECTS_SOURCE_FILES
  src/joints_estimator.cpp
  src/madgwick_filter.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

# NODE: Lower level position controls
add_executable(${PROJECT_NAME}_joints_estimator src/joints_estimator_node.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_joints_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_joints_estimator ${catkin_LIBRARIES} ${ROS_LIBRARIES})