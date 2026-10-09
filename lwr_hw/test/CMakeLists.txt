# Test-only targets; included from ../CMakeLists.txt when LWR_BUILD_TESTS=ON.
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_lwr_hw test_lwr_hw.cpp)
target_link_libraries(test_lwr_hw lwr_hw lwr_hw_gazebo_plugin ignition-gazebo6::core)
ament_target_dependencies(test_lwr_hw ${hw_dependencies} gz_ros2_control)
# Exercise the real FRI adapter/packet code with mocked socket syscalls.
# This target cannot contact a robot, even when activation is tested.
ament_add_gtest(test_lwr_fri test_lwr_fri.cpp ${PROJECT_SOURCE_DIR}/src/fri/friremote.cpp ${PROJECT_SOURCE_DIR}/src/fri/friudp.cpp)
target_compile_definitions(test_lwr_fri PRIVATE HAVE_SOCKLEN_T)
target_link_libraries(test_lwr_fri lwr_hw)
ament_target_dependencies(test_lwr_fri ${hw_dependencies})
target_link_options(test_lwr_fri PRIVATE
  -Wl,--wrap=socket -Wl,--wrap=bind -Wl,--wrap=setsockopt -Wl,--wrap=sendto
  -Wl,--wrap=recvfrom -Wl,--wrap=close -Wl,--wrap=gethostbyname)
