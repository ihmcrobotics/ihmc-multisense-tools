cmake_minimum_required(VERSION 2.8.3)
project(scan_to_cloud)

find_package(catkin REQUIRED COMPONENTS
  laser_assembler
  laser_filters
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  PointCloud2WithSource.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(periodic_assembler src/periodic_assembler.cpp)
 
target_link_libraries(periodic_assembler
   ${catkin_LIBRARIES}
 )

