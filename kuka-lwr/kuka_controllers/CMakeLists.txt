cmake_minimum_required(VERSION 3.8)
project(kuka_controllers)

# ROS 1: add_definitions(-fpermissive -std=c++11); rclcpp needs C++17
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fpermissive)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies
  controller_interface
  hardware_interface
  control_msgs
  geometry_msgs
  visualization_msgs
  control_toolbox
  realtime_tools
  urdf
  rclcpp
  rclcpp_lifecycle
  pluginlib
  kdl_parser
  orocos_kdl_vendor
  sensor_msgs
  std_msgs
  lwr_hw
  lwr_controllers
)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
 src/basic_joint_controller.cpp
 src/hierarchical_controller.cpp
 src/very_basic_joint_controller.cpp
 src/KinematicChainControllerBase.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
pluginlib_export_plugin_description_file(controller_interface kuka_controllers_plugins.xml)

add_executable(rcm_marker src/rcm_marker.cpp)
ament_target_dependencies(rcm_marker rclcpp visualization_msgs geometry_msgs)

# ROS 1 installed include/${PROJECT_NAME}/, which does not exist (the headers are in include/)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
# ROS 1 did not install rcm_marker (rosrun found it in devel space)
install(TARGETS rcm_marker
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES kuka_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
