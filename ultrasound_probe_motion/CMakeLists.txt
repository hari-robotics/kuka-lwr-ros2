cmake_minimum_required(VERSION 3.8)
project(ultrasound_probe_motion)
# ROS 1: -std=c++11; rclcpp needs C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lwr_controllers REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS 1: add_action_files(ProbeStep.action) + generate_messages(actionlib_msgs geometry_msgs)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ProbeStep.action"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(${PROJECT_NAME}_lib src/quintic_duration.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_executable(probe_relative_motion_node src/probe_relative_motion_node.cpp)
ament_target_dependencies(probe_relative_motion_node rclcpp rclcpp_action std_msgs geometry_msgs lwr_controllers)
target_link_libraries(probe_relative_motion_node ${PROJECT_NAME}_lib Eigen3::Eigen "${typesupport_target}")

add_executable(probe_scan_motion_node src/probe_scan_motion_node.cpp)
ament_target_dependencies(probe_scan_motion_node rclcpp rclcpp_action std_msgs geometry_msgs lwr_controllers)
target_link_libraries(probe_scan_motion_node ${PROJECT_NAME}_lib Eigen3::Eigen "${typesupport_target}")

# ROS 1 library target ${PROJECT_NAME} (libultrasound_probe_motion); in ROS 2 that target name is
# taken by the generated interfaces, so the library is ultrasound_probe_motion_lib
install(TARGETS ${PROJECT_NAME}_lib EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS probe_relative_motion_node probe_scan_motion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime rclcpp rclcpp_action std_msgs geometry_msgs lwr_controllers Eigen3)
ament_package()
