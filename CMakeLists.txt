cmake_minimum_required(VERSION 2.8.3)
project(leap_motion_demo)

find_package(catkin REQUIRED)

catkin_package()

# Install
install(DIRECTORY launch, scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
