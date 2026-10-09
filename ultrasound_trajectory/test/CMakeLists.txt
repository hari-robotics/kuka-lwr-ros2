# Test-only targets; included from ../CMakeLists.txt when LWR_BUILD_TESTS=ON.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
ament_add_gtest(test_trajectory_math test_math.cpp ${PROJECT_SOURCE_DIR}/src/se3_trajectory.cpp)
target_include_directories(test_trajectory_math PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_trajectory_math Eigen3::Eigen ${orocos_kdl_LIBRARIES})
ament_target_dependencies(test_trajectory_math orocos_kdl_vendor)
ament_add_pytest_test(test_ros2_nodes test_nodes.py TIMEOUT 150
  APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs)
