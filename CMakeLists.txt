idf_component_register(SRCS "lsm6dsv32x.cpp" "lsm6dsv32x_reg.c"
                    REQUIRES driver vectors esp_timer
                    INCLUDE_DIRS "." "include")
