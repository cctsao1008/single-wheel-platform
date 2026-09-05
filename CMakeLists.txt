cmake_minimum_required(VERSION 3.20)

project(single_wheel_platform VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Architecture contracts are exposed as interface targets while the
# hardware implementation and control algorithms are brought online.
add_library(swp_control INTERFACE)
target_include_directories(swp_control INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/control
)

add_library(swp_platform_api INTERFACE)
target_include_directories(swp_platform_api INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/api
)
