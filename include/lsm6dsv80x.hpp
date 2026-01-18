#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "vectors.h"
#include "Quaternion.hpp"
#include "lsm6dsv80x_reg.h"

typedef struct {
    uint16_t sampleRate;
    uint8_t accelRange;
    uint16_t gyroRange;
    lsm6dsv80x_fifo_xl_batch_t batching;
    uint16_t fifoWatermark;
    lsm6dsv80x_fifo_mode_t fifoMode;
    uint32_t timeout;
} lsm6dsv80x_cfg_t;

typedef struct {
    bool tap_x;
    bool tap_y;
    bool tap_z;
    bool tap_single;
    bool tap_double;
    bool tap_sign;
    bool sleep_change;
} lsm6dsv80x_sources_t;

typedef struct {
    Vector3<float> acc;
    Vector3<float> gyro;
    Vector3<float> gravity;
    Quaternion<float> q;
    float timestamp;
} fifo_element_t;

void lsm6dsv80x_init_spi(spi_device_handle_t *dev_handle);
void lsm6dsv80x_init();
void lsm6dsv80x_config(lsm6dsv80x_cfg_t *cfg);
void lsm6dsv80x_start_sampling(bool start);
void lsm6dsv80x_start_tapping(bool start);
float lsm6dsv80x_get_timestamp_resolution();
int lsm6dsv80x_fifo_data_available();
int lsm6dsv80x_fifo_read_element(lsm6dsv80x_fifo_out_raw_t &f_data);
void lsm6dsv80x_fifo_process_xl(lsm6dsv80x_fifo_out_raw_t &f_data, Vector3<float> &acc);
void lsm6dsv80x_fifo_process_gyro(lsm6dsv80x_fifo_out_raw_t &f_data, Vector3<float> &gyro);
void lsm6dsv80x_fifo_process_gravity(lsm6dsv80x_fifo_out_raw_t &f_data, Vector3<float> &gravity);
void lsm6dsv80x_fifo_process_sflp_game_rotation(lsm6dsv80x_fifo_out_raw_t &f_data1, lsm6dsv80x_fifo_out_raw_t &f_data2, Quaternion<float> &q);
void lsm6dsv80x_fifo_process_timestamp(lsm6dsv80x_fifo_out_raw_t &f_data, float &timestamp);

void lsm6dsv80x_read_sources(lsm6dsv80x_sources_t *sources);
void lsm6dsv80x_sleep();
void lsm6dsv80x_set_debug(bool d);
