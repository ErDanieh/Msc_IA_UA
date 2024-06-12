cmake_minimum_required(VERSION 3.1.3)
project(robotica_inteligente)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/spawn_random_position.py 
  scripts/object_localization.py 
  scripts/move_robot_arm.py 
  scripts/blue_navigation.py
  scripts/obstacle_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
