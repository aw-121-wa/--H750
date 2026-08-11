function(add_host_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/stubs
        ${PROJECT_SOURCE_DIR}
    )
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(chassis_kinematics_test
    control/chassis/chassis_kinematics_test.c
    ../control/chassis/chassis_kinematics.c
    ../devices/motor/zdt_x42s/zdt_x42s_protocol.c
)

add_host_test(zdt_x42s_driver_test
    devices/motor/zdt_x42s/zdt_x42s_driver_test.c
    ../devices/motor/zdt_x42s/zdt_x42s.c
    ../devices/motor/zdt_x42s/zdt_x42s_protocol.c
)

add_host_test(chassis_driver_test
    control/chassis/chassis_driver_test.c
    ../control/chassis/chassis.c
    ../control/chassis/chassis_kinematics.c
)

add_host_test(chassis_smoke_test
    app/diagnostics/chassis_smoke_test_test.c
    ../app/diagnostics/chassis_smoke_test.c
)

add_host_test(dwt_delay_test
    platform/time/dwt_delay_test.c
    ../platform/time/dwt_delay.c
    ../platform/time/delay.c
)
target_compile_definitions(dwt_delay_test PRIVATE
    DWT_DELAY_CYCLE_COUNTER=test_read_cycles
)

add_host_test(hwt101_parser_test
    devices/imu/hwt101/hwt101_parser_test.c
    ../devices/imu/hwt101/hwt101_parser.c
)
target_link_libraries(hwt101_parser_test PRIVATE m)

add_host_test(heading_control_test
    control/heading/heading_control_test.c
    ../control/heading/heading_control.c
    ../control/common/pid.c
)
target_link_libraries(heading_control_test PRIVATE m)
