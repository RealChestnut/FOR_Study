cmake_minimum_required(VERSION 2.8.3)
project(serial_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_magnetic_node src/serial_magnetic_node.cpp)

target_link_libraries(serial_magnetic_node
  ${catkin_LIBRARIES}
)

