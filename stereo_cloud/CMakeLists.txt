cmake_minimum_required(VERSION 2.8.3)
project(stereo_cloud)

find_package(catkin REQUIRED COMPONENTS
  laser_assembler
  laser_filters
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  scan_to_cloud
)

#add_message_files(
#  DIRECTORY msg
#  FILES
#  PointCloud2WithSource.msg
#)

#generate_messages(DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  scan_to_cloud
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(periodic_assembler src/periodic_assembler.cpp)
 
target_link_libraries(periodic_assembler
   ${catkin_LIBRARIES}
 )

install(TARGETS periodic_assembler
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)

catkin_install_python(PROGRAMS scripts/voxelCloudPublisher.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
