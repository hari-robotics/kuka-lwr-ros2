cmake_minimum_required(VERSION 3.8)
project(sriforcesensor)

# ROS 1: set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}" ) and add_compile_options(-std=c++14);
# rclcpp (Humble) requires C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lwr_controllers REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Force sensor TCP client (hardware communication, copied unchanged)
add_library(TCPClient2 SHARED src/TCPClient.cpp src/TCPClient.h)
target_link_libraries(TCPClient2 Eigen3::Eigen)

add_executable(forcesensor src/sriforcesensor.cpp)
ament_target_dependencies(forcesensor rclcpp std_msgs geometry_msgs lwr_controllers)
target_link_libraries(forcesensor TCPClient2 Eigen3::Eigen)

install(TARGETS TCPClient2 LIBRARY DESTINATION lib)
install(TARGETS forcesensor DESTINATION lib/${PROJECT_NAME})
# ROS 1 ran these with rosrun from the source tree (executable files); ROS 2 installs them
install(PROGRAMS
  src/ANN/tool_comp_ros.py
  src/ANN/record_calib.py
  src/ANN/tool_comp.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
