cmake_minimum_required(VERSION 2.8.3)
project(lslidar_x10_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LslidarX10Packet.msg
  LslidarX10Point.msg
  LslidarX10Scan.msg
  LslidarX10Sweep.msg
  LslidarX10Difop.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
