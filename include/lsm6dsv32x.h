#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"
#include "vectors.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "lsm6dsv32x_reg.h"

typedef struct {
    uint16_t sampleRate;
    uint8_t accelRange;
    uint16_t gyroRange;
    lsm6dsv32x_fifo_xl_batch_t batching;
    uint16_t fifoWatermark;
    lsm6dsv32x_fifo_mode_t fifoMode;
    uint32_t timeout;
} lsm6dsv32x_cfg_t;

typedef struct {
    bool tap_x;
    bool tap_y;
    bool tap_z;
    bool tap_single;
    bool tap_double;
    bool tap_sign;
    bool sleep_change;
} lsm6dsv32x_sources_t;

typedef struct {
    vector3_t acc;
    vector3_t gyro;
    vector3_t gravity;
    quaternion_t q;
    float timestamp;
} fifo_element_t;

void lsm6dsv32x_init_i2c(i2c_master_dev_handle_t dev_handle);
void lsm6dsv32x_init();
void lsm6dsv32x_config(lsm6dsv32x_cfg_t *cfg);
void lsm6dsv32x_start_sampling(bool start);
void lsm6dsv32x_start_tapping(bool start);
float lsm6dsv32x_get_timestamp_resolution(float sample_rate);
int lsm6dsv32x_fifo_data_available();
int lsm6dsv32x_read_fifo_element(fifo_element_t *fifo);
void lsm6dsv32x_read_sources(lsm6dsv32x_sources_t *sources);
void lsm6dsv32x_sleep();


#ifdef __cplusplus
}
#endif