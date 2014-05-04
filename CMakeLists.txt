cmake_minimum_required(VERSION 2.8.3)
project(missionImpossible)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp  sensor_msgs image_transport cv_bridge)

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES std_msgs)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Opencv_LIBS}
)


## Declare a cpp executable
# add_executable(missinImpossible_node src/missinImpossible.cpp)
 add_executable(missinImpossible_node src/main.cpp)

 target_link_libraries(missinImpossible_node ${catkin_LIBRARIES} zbar)
