cmake_minimum_required(VERSION 3.8)
project(ultrasound_trajectory)

# ROS 1: add_definitions (-fpermissive -std=c++11)
add_compile_options(-fpermissive)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
set(dependencies rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs control_msgs std_srvs lwr_controllers
  message_filters kdl_parser orocos_kdl_vendor urdf)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
# ROS 1: find_package (OpenCV 3.3.1 REQUIRED); Humble ships OpenCV 4 (no OpenCV call in the sources)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(ultrasound_traj_node src/ultrasound_traj_node.cpp)
add_executable(linear_traj_node src/linear_traj_node.cpp)
add_executable(linear_traj_node_Luca src/linear_traj_node_Luca.cpp)
add_executable(pose_initialization src/pose_initialization.cpp)
add_executable(ultrasound_traj_cartesian_node src/ultrasound_traj_cartesian_node.cpp)
add_executable(pose_init src/pose_init.cpp)
add_executable(ee_incremental_motion src/ee_incremental_motion.cpp)
add_executable(ee_incremental_impedance_node src/ee_incremental_impedance_node.cpp)
add_executable(tool_relative_motion_node
  src/tool_relative_motion_node.cpp
  src/se3_trajectory.cpp
  src/ik_safety_gate.cpp
  src/motion_state_machine.cpp
)

set(executables ultrasound_traj_node linear_traj_node linear_traj_node_Luca pose_initialization
  ultrasound_traj_cartesian_node pose_init ee_incremental_motion ee_incremental_impedance_node
  tool_relative_motion_node)
foreach(executable IN LISTS executables)
  ament_target_dependencies(${executable} ${dependencies})
  target_link_libraries(${executable} Eigen3::Eigen)
endforeach()

install(TARGETS ${executables} DESTINATION lib/${PROJECT_NAME})
ament_package()
