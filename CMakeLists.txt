cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

project(i2c_slave_test)


pico_sdk_init()

add_executable(i2c_slave_test
        i2c-slave.cpp
        )

target_link_libraries(i2c_slave_test 
pico_stdlib 
hardware_i2c 
pico_multicore )

pico_enable_stdio_usb(i2c_slave_test 1)
pico_enable_stdio_uart(i2c_slave_test 0)

pico_add_extra_outputs(i2c_slave_test)

add_custom_command(TARGET i2c_slave_test
    # Run after all other rules within the target have been executed
    POST_BUILD
    COMMAND pre-load
    COMMENT "Loading uf2 file onto PICO..."
    VERBATIM
)

