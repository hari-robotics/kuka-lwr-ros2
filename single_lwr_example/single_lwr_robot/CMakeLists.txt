cmake_minimum_required(VERSION 3.8)
project(single_lwr_robot)
find_package(ament_cmake REQUIRED)
install(DIRECTORY robot DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Tests live in test/ and stay out of the default (user) build and install space.
# Build them explicitly: colcon build --cmake-args -DLWR_BUILD_TESTS=ON
option(LWR_BUILD_TESTS "Build the test suite in test/ (also requires BUILD_TESTING)" OFF)
if(BUILD_TESTING AND LWR_BUILD_TESTS)
  add_subdirectory(test)
endif()
ament_package()
