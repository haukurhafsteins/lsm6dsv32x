#pragma once
#include "imu_lsm6dsv.hpp"
// Run the identical baseline scenario through the neutral API as well as the
// compatibility API. These adapters are test-only and perform no I/O themselves.
inline void neutral_config(lsm6dsv80x_cfg_t *c)
{
    imu::Config n;
    n.sampleRate = static_cast<imu::Rate>(c->sampleRate);
    n.accelRange = static_cast<imu::AccelRange>(c->accelRange);
    n.gyroRange = static_cast<imu::GyroRange>(c->gyroRange);
    n.fifoWatermark = c->fifoWatermark;
    n.fifoTemperature = c->fifoTemperature;
    if (!imu::config(n))
        std::abort();
}
inline imu::FifoWord neutral_word(const lsm6dsv80x_fifo_out_raw_t &n)
{
    imu::FifoWord w;
    w.tag = static_cast<imu::Tag>(n.tag);
    w.counter = n.cnt;
    std::memcpy(w.data.data(), n.data, 6);
    return w;
}
inline int neutral_read(lsm6dsv80x_fifo_out_raw_t &n)
{
    imu::FifoWord w;
    auto result = imu::fifo_read_element(w);
    n.tag = static_cast<decltype(n.tag)>(w.tag);
    n.cnt = w.counter;
    std::memcpy(n.data, w.data.data(), 6);
    return result;
}
inline void neutral_rotation(lsm6dsv80x_fifo_out_raw_t &a, lsm6dsv80x_fifo_out_raw_t &b,
                             Quaternion<float> &q)
{
    imu::FifoWord words[]{neutral_word(a), neutral_word(b)};
    imu::RotationDecoder decoder;
    if (decoder.decode(words, 2, q) != imu::DecodeResult::Ready)
        std::abort();
}
#define lsm6dsv80x_init_spi imu::init_spi
#define lsm6dsv80x_init imu::init
#define lsm6dsv80x_config neutral_config
#define lsm6dsv80x_start_sampling imu::start_sampling
#define lsm6dsv80x_fifo_data_available imu::fifo_data_available
#define lsm6dsv80x_fifo_overrun_count imu::fifo_overrun_count
#define lsm6dsv80x_fifo_read_element neutral_read
#define lsm6dsv80x_fifo_process_sflp_game_rotation neutral_rotation
#define lsm6dsv80x_fifo_process_xl(w, v) imu::process_xl(neutral_word(w), v)
#define lsm6dsv80x_fifo_process_gyro(w, v) imu::process_gyro(neutral_word(w), v)
#define lsm6dsv80x_fifo_process_gravity(w, v) imu::process_gravity(neutral_word(w), v)
#define lsm6dsv80x_fifo_process_temperature(w, v) imu::process_temperature(neutral_word(w), v)
#define lsm6dsv80x_fifo_process_timestamp(w, v) imu::process_timestamp(neutral_word(w), v)
