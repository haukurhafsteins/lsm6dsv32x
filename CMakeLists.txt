# Existing source-list consumers may keep lsm6dsv80x.cpp + lsm6dsv80x_reg.c.
# New consumers set this cache variable and propagate the same define publicly.
set(IMU_LSM6DSV_PART "80" CACHE STRING "LSM6DSV part: 80 or 32")
set_property(CACHE IMU_LSM6DSV_PART PROPERTY STRINGS 80 32)
if(NOT IMU_LSM6DSV_PART STREQUAL "80" AND NOT IMU_LSM6DSV_PART STREQUAL "32")
    message(FATAL_ERROR "IMU_LSM6DSV_PART must be 80 or 32")
endif()
idf_component_register(SRCS "lsm6dsv80x.cpp" "lsm6dsv${IMU_LSM6DSV_PART}x_reg.c"
                    REQUIRES rtos vectors
                    INCLUDE_DIRS "." "include")
target_compile_definitions(${COMPONENT_LIB} PUBLIC IMU_LSM6DSV_PART=${IMU_LSM6DSV_PART})
