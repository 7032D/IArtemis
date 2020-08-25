cmake_minimum_required(VERSION 2.8.3)
project(c_iartemis_pid)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  vfw_int32.msg
  vfw_float32.msg
)

add_service_files(
  FILES
  k_coeff.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CMAKE_CXX_FLAGS " -Wall -std=c++11  ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
CATKIN_DEPENDS message_runtime
)


# XR Mise en commentaire suite creation du pkg c_robot
#add_executable(c_pid_node src/c_pid_node.cpp)
#target_link_libraries(c_pid_node ${catkin_LIBRARIES})
#add_dependencies(c_pid_node c_iartemis_pid_generate_messages_cpp)
#
#add_executable(xr_pid_node src/xr_pid_node.cpp)
#target_link_libraries(xr_pid_node ${catkin_LIBRARIES})
#
#add_executable(xr_pid_node_v1 src/xr_pid_node_v1.cpp)
#target_link_libraries(xr_pid_node_v1 ${catkin_LIBRARIES})
