cmake_minimum_required(VERSION 2.8.3)
project(transform_provider)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf tf2 geometry_msgs message_generation
)

add_service_files(
  FILES
  TransformProvider.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs tf geometry_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_provider_node src/transform_provider_node.cpp)

add_dependencies(transform_provider_node transform_provider_generate_messages_cpp)

target_link_libraries(transform_provider_node
   ${catkin_LIBRARIES}
)

install(TARGETS transform_provider_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)