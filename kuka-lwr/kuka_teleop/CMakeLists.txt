cmake_minimum_required(VERSION 3.8)
project(kuka_teleop)

find_package(ament_cmake REQUIRED)

# ROS 1 ran the scripts with rosrun from the source tree (executable files); ROS 2 installs them
# under their original names: ros2 run kuka_teleop kuka_teleop.py
install(PROGRAMS
  scripts/kuka_teleop.py
  scripts/joint_traj.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
