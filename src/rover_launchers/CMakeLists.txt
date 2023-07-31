cmake_minimum_required(VERSION 2.8.3)
project(rover_launchers)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)

find_package(catkin REQUIRED)
# find_package(sb_utils REQUIRED)


catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

