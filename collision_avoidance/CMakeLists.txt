cmake_minimum_required(VERSION 2.8.3)
project(collision_avoidance)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(collision_avoidance src/collision_avoidance.cpp)

target_link_libraries(collision_avoidance
  ${catkin_LIBRARIES}
)

