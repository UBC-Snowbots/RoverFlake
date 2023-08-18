cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#arm_firmware v2
project(arm_firmware)


target_include_directories(app PRIVATE 
    include
    src
)

#telling it what targets to aim for
target_sources(app PRIVATE src/main.cpp src/Axis.cpp )

