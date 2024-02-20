cmake_minimum_required(VERSION 2.8.3)
project(aecom_commander)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
  geometry_msgs
  sensor_msgs
  #nlink_parser
  message_generation
  tf
  #apriltag_ros
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    )
## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)
 find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES aecom_commander
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 ${catkin_INCLUDE_DIRS}
)

# add_library($aecom_commander
#   src/aecom_commander/aecom_commander.cpp
# )

# add_dependencies(aecom_commander ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp})

 add_executable(aecom_commander_drone aecom_commanderfinal.cpp)
 add_executable(aecom_commander_robot_arm aecom_commander_ugv_robot_arm.cpp)
 add_executable(aecom_commander_ugv_navigation aecom_commander_ugv_controller.cpp)
#  add_executable(aecom_commander_safmc aecom_commander_safmc.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

#add_dependencies(aecom_commander_node aecom_commander)

target_link_libraries(
  aecom_commander_drone
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
#  ${system_lib_LIBRARIES}
  ${mavros_LIBRARIES}
)
target_link_libraries(
  aecom_commander_robot_arm
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
#  ${system_lib_LIBRARIES}
  ${mavros_LIBRARIES}
)
target_link_libraries(
  aecom_commander_ugv_navigation
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
#  ${system_lib_LIBRARIES}
  ${mavros_LIBRARIES}
)
#target_link_libraries(
#  aecom_commander_safmc
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${system_lib_LIBRARIES}
#  ${mavros_LIBRARIES}
#)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aecom_commander.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
