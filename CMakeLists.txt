cmake_minimum_required(VERSION 2.8.3)
project(brunel_hand_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(diagnostics src/diagnostics.cpp)
target_link_libraries(diagnostics ${catkin_LIBRARIES})
