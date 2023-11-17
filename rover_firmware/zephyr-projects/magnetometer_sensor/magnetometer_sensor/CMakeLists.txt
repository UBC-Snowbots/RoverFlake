# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(magnetometer_sensor)

# # Include custom sensor driver directory
# zephyr_include_directories(drivers/lsm9ds1_magn)

# Add your source files here
target_sources(app PRIVATE src/main.c) #drivers/lsm9ds1_magn/lsm9ds1_magn.c)