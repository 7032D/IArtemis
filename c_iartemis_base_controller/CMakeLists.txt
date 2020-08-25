cmake_minimum_required(VERSION 2.8.3)
project(c_iartemis_base_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  vfw_int32.msg
  vfw_float32.msg
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
#add_executable(dcmc_rv_motor_controller src/dcmc_rv_motor_controller.cpp)
#target_link_libraries(dcmc_rv_motor_controller ${catkin_LIBRARIES})
#add_dependencies(dcmc_rv_motor_controller c_iartemis_base_controller_generate_messages_cpp)
