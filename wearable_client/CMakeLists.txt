cmake_minimum_required(VERSION 2.8.3)
project(wearable_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wearable_client_node src/wearable_client_node.cpp)

target_link_libraries(wearable_client_node
   ${catkin_LIBRARIES}
)

