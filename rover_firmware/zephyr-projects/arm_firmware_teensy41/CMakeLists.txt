cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#arm_firmware v2
project(arm_firmware)


target_include_directories(app PRIVATE 
    include
    src
)

#telling it what targets to aim for
FILE(GLOB app_sources src/*.cpp)
target_sources(app PRIVATE ${app_sources})
