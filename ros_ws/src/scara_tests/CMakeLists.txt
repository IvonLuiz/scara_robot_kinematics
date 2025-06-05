cmake_minimum_required(VERSION 3.0.2)
project(scara_tests)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/test_fk.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
