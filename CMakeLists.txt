cmake_minimum_required(VERSION 2.8.3)
project(mission_impossible)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  joy
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mission_impossible
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport joy roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Opencv_LIBS}
)


add_executable(ardrone_controller_node src/ardrone_controller.cpp)

target_link_libraries(ardrone_controller_node ${catkin_LIBRARIES} )

add_executable(missionImpossible_node src/main.cpp)

target_link_libraries(missionImpossible_node ${catkin_LIBRARIES} zbar)

