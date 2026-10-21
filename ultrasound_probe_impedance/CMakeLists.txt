cmake_minimum_required(VERSION 3.8)
project(ultrasound_probe_impedance)
# ROS 1: -std=c++11; rclcpp needs C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lwr_controllers REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(probe_incremental_impedance_node src/probe_incremental_impedance_node.cpp)
ament_target_dependencies(probe_incremental_impedance_node rclcpp std_msgs geometry_msgs std_srvs lwr_controllers)
target_link_libraries(probe_incremental_impedance_node Eigen3::Eigen)

install(TARGETS probe_incremental_impedance_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(probe_incremental_force_node src/probe_incremental_force_node.cpp)
ament_target_dependencies(probe_incremental_force_node rclcpp std_msgs geometry_msgs std_srvs lwr_controllers)
target_link_libraries(probe_incremental_force_node Eigen3::Eigen)

install(TARGETS probe_incremental_force_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
