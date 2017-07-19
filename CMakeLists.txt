cmake_minimum_required(VERSION 2.8.3)
project(brunel_hand_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES
  FingerPose.msg
  RawCommand.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(proxy src/proxy.cpp)
target_link_libraries(proxy ${catkin_LIBRARIES})
