cmake_minimum_required(VERSION 2.8.3)
project(serial_ae)
#set(serial_DIR /usr/local)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #serial
  std_msgs
)
#add_subdirectory(extern/serial)
#include_directories(extern/serial/include)
catkin_package(
  CATKIN_DEPENDS
    #serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_ae_node src/serial_ae_node.cpp)

target_link_libraries(serial_ae_node
  ${catkin_LIBRARIES}
  serial
)

