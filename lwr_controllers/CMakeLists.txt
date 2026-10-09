cmake_minimum_required(VERSION 3.8)
project(lwr_controllers)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
set(controller_dependencies controller_interface hardware_interface rclcpp rclcpp_lifecycle pluginlib
  control_toolbox realtime_tools urdf kdl_parser orocos_kdl_vendor angles geometry_msgs
  sensor_msgs std_msgs visualization_msgs control_msgs tf2 tf2_ros tf2_kdl)
foreach(dependency IN LISTS controller_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseRPY.msg" "msg/RPY.msg" "msg/MultiPriorityTask.msg" "msg/ArmState.msg"
  "msg/CartesianImpedancePoint.msg" "msg/Stiffness.msg" "srv/SetCartesianImpedanceCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
add_library(${PROJECT_NAME}_plugins SHARED
  src/joint_impedance_controller.cpp src/inverse_dynamics_controller.cpp
  src/computed_torque_controller.cpp src/one_task_inverse_kinematics.cpp
  src/multi_task_priority_inverse_kinematics.cpp src/multi_task_priority_inverse_dynamics.cpp
  src/minimum_effort_inverse_dynamics.cpp src/backstepping_controller.cpp
  src/dynamic_sliding_mode_controller.cpp src/dynamic_sliding_mode_controller_task_space.cpp
  src/one_task_inverse_dynamics_jl.cpp src/gravity_compensation.cpp src/arm_state_controller.cpp
  src/cartesian_impedance_controller.cpp src/KinematicChainControllerBase.cpp)
target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_plugins ${controller_dependencies})
target_link_libraries(${PROJECT_NAME}_plugins Eigen3::Eigen)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_plugins "${typesupport_target}")
pluginlib_export_plugin_description_file(controller_interface lwr_controllers_plugins.xml)
install(TARGETS ${PROJECT_NAME}_plugins EXPORT export_lwr_controllers
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_lwr_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${controller_dependencies} rosidl_default_runtime Eigen3)
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_lifecycle test/test_controller_lifecycle.cpp
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs)
  ament_target_dependencies(test_controller_lifecycle pluginlib controller_interface hardware_interface rclcpp rclcpp_lifecycle)
  add_executable(test_lwr_plugins test/test_plugins.cpp)
  ament_target_dependencies(test_lwr_plugins pluginlib controller_interface)
  ament_add_test(plugin_loading COMMAND $<TARGET_FILE:test_lwr_plugins>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
