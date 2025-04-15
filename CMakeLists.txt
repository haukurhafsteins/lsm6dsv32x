idf_component_register(SRCS "lsm6dsv32x.c" "lsm6dsv32x_reg.c"
                    REQUIRES driver vectors
                    INCLUDE_DIRS "." "include")
