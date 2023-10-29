cmake_minimum_required(VERSION 3.0.2)
project(PseudoGasSensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  sensor_msgs
  gazebo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PseudoGasSensor
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/PseudoGasSensor.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})