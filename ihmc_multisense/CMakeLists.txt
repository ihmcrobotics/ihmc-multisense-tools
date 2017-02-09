cmake_minimum_required(VERSION 2.8.3)
project(ihmc_multisense)

find_package(catkin REQUIRED COMPONENTS
  laser_assembler
  laser_filters
  roscpp
  sensor_msgs
  std_msgs
  multisense_bringup
  ihmc_ros_common
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)