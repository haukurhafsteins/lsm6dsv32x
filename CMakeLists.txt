idf_component_register(SRCS "lsm6dsv80x.cpp" "lsm6dsv80x_reg.c"
                    REQUIRES rtos vectors
                    INCLUDE_DIRS "." "include")
