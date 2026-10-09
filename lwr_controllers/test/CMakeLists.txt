# Test-only targets; included from ../CMakeLists.txt when LWR_BUILD_TESTS=ON.
find_package(ament_cmake_test REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_controller_lifecycle test_controller_lifecycle.cpp
  APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs)
ament_target_dependencies(test_controller_lifecycle pluginlib controller_interface hardware_interface rclcpp rclcpp_lifecycle)
add_executable(test_lwr_plugins test_plugins.cpp)
ament_target_dependencies(test_lwr_plugins pluginlib controller_interface)
ament_add_test(plugin_loading COMMAND $<TARGET_FILE:test_lwr_plugins>
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
