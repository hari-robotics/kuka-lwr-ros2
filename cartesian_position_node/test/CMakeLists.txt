# Test-only targets; included from ../CMakeLists.txt when LWR_BUILD_TESTS=ON.
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(test_cartesian_position test_cartesian_position.py TIMEOUT 90
  APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs)
