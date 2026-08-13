cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

option(BUILD_HOST_TESTS "Build native unit tests instead of firmware" OFF)
if(BUILD_HOST_TESTS)
    project(GongxunH750HostTests LANGUAGES C)
    enable_testing()
    add_subdirectory(tests)
    return()
endif()

project(GongxunH750 LANGUAGES C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

target_sources(${PROJECT_NAME} PRIVATE
    control/chassis/chassis.c
    control/chassis/chassis_kinematics.c
    control/common/pid.c
    control/heading/heading_control.c
    control/localization/localization.c
    control/localization/localization_service.c
    control/navigation/navigation.c
    devices/imu/hwt101/hwt101.c
    devices/imu/hwt101/hwt101_parser.c
    devices/motor/zdt_x42s/zdt_x42s.c
    devices/motor/zdt_x42s/zdt_x42s_protocol.c
    platform/communication/host_link.c
    platform/communication/host_dispatcher.c
    platform/communication/host_protocol.c
    platform/communication/uart_callbacks.c
    platform/time/delay.c
    platform/time/dwt_delay.c
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR})

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
target_link_libraries(${PROJECT_NAME} stm32cubemx)
