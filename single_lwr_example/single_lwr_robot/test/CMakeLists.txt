# Test-only targets; included from ../CMakeLists.txt when LWR_BUILD_TESTS=ON.
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(model test_model.py TIMEOUT 60)
