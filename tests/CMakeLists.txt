cmake_minimum_required(VERSION 3.16)
project(imu_driver_host_tests LANGUAGES C CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(URU_COMPONENTS_DIR "" CACHE PATH "URU components directory containing rtos and vectors")
if(NOT EXISTS "${URU_COMPONENTS_DIR}/rtos/include/rtos/Spi.hpp")
    message(FATAL_ERROR "Set URU_COMPONENTS_DIR to initialized URU firmware components")
endif()
add_executable(fifo_temperature_test fifo_temperature_test.cpp ../lsm6dsv80x_reg.c)
target_include_directories(fifo_temperature_test PRIVATE
    .. ../include
    "${URU_COMPONENTS_DIR}/rtos/include"
    "${URU_COMPONENTS_DIR}/vectors/include")
# Unused platform entry points are outside this register-level host test.
target_compile_options(fifo_temperature_test PRIVATE -ffunction-sections -fdata-sections)
target_link_options(fifo_temperature_test PRIVATE -Wl,--gc-sections)
add_test(NAME fifo_temperature COMMAND fifo_temperature_test)
