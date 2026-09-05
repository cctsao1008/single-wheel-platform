cmake_minimum_required(VERSION 3.20)

project(single_wheel_platform VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Architecture contracts are exposed independently from concrete board code.
add_library(swp_control INTERFACE)
target_include_directories(swp_control INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/control
)

add_library(swp_platform_api INTERFACE)
target_include_directories(swp_platform_api INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/api
)

# Portable reference-device driver. Platform transport is injected by the
# application; the driver has no STM32-specific dependency.
add_library(swp_mpu6050 STATIC
    drivers/mpu6050/mpu6050.c
)
target_include_directories(swp_mpu6050 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/mpu6050
)

# Concrete STM32F103 reference-platform implementation. These sources use
# memory-mapped STM32F103 peripheral registers but no controller types.
add_library(swp_platform_stm32f103 STATIC
    platform/stm32f103/board_time_stm32f103.c
    platform/stm32f103/board_i2c_stm32f103.c
    platform/stm32f103/board_motor_stm32f103.c
    platform/stm32f103/board_encoder_stm32f103.c
)
target_include_directories(swp_platform_stm32f103 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/api
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32f103
)
target_link_libraries(swp_platform_stm32f103 PUBLIC swp_platform_api)

# Application-side binding between the portable MPU6050 driver and the
# selected board services.
add_library(swp_app_bindings STATIC
    app/mpu6050_platform_binding.c
)
target_include_directories(swp_app_bindings PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/mpu6050
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/api
)
target_link_libraries(swp_app_bindings PUBLIC
    swp_mpu6050
    swp_platform_api
)
