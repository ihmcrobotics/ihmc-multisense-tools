cmake_minimum_required(VERSION 2.8.3)
project(scan_to_cloud)

find_package(catkin REQUIRED COMPONENTS
  laser_assembler
  laser_filters
  roscpp
  sensor_msgs
  std_msgs
  multisense_bringup
)

catkin_package(

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

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        FILES_MATCHING PATTERN "*.yaml"
)