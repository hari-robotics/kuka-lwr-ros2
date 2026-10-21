cmake_minimum_required(VERSION 3.8)
project(frame_grabber)

find_package(ament_cmake REQUIRED)

# ROS 1 ran the scripts with rosrun from the source tree (executable files); ROS 2 installs them
install(PROGRAMS
  src/cameratoROS.py
  src/cameratoROS_cropped.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
