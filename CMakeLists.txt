cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

project(i2c_slave_test)

pico_sdk_init()

add_executable(i2c_slave_test midiFt_pico.cpp include/midiFeetPICO_lib.cpp)

target_link_libraries(i2c_slave_test 
pico_stdlib 
hardware_i2c 
pico_multicore
)

add_library(midiFeetPICO_lib INTERFACE)
target_sources(midiFeetPICO_lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mysource1.c
    ${CMAKE_CURRENT_LIST_DIR}/mysource2.c
)

target_link_libraries(midiFeetPICO_lib INTERFACE 
pico_stdlib 
hardware_i2c 
pico_multicore )

target_include_directories(midiFeetPICO_lib INTERFACE S{CMAKE_CURRENT_LIST_DIR}/include)

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

