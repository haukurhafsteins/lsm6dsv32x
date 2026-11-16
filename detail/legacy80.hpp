// Source-compatible 80X entry points. No 80X API is linked in a 32X image.
#include "../include/lsm6dsv80x.hpp"
void lsm6dsv80x_init_spi(rtos::SpiDevice *d) { imu::init_spi(d); }
void lsm6dsv80x_init() { (void)imu::init(); }
void lsm6dsv80x_config(lsm6dsv80x_cfg_t *c)
{
    if (!c)
        return;
    DriverConfig n{c->sampleRate,    c->accelRange, c->gyroRange, c->batching,
                   c->fifoWatermark, c->fifoMode,   c->timeout,   c->fifoTemperature};
    impl_config(&n);
}
void lsm6dsv80x_start_sampling(bool v) { imu::start_sampling(v); }
void lsm6dsv80x_start_tapping(bool v) { imu::start_tapping(v); }
float lsm6dsv80x_get_timestamp_resolution() { return imu::get_timestamp_resolution(); }
int lsm6dsv80x_fifo_data_available() { return imu::fifo_data_available(); }
uint32_t lsm6dsv80x_fifo_overrun_count() { return imu::fifo_overrun_count(); }
int lsm6dsv80x_fifo_read_element(lsm6dsv80x_fifo_out_raw_t &w) { return impl_fifo_read_element(w); }
void lsm6dsv80x_fifo_process_xl(lsm6dsv80x_fifo_out_raw_t &w, Vector3<float> &v)
{
    if (driver_to_mg)
        impl_fifo_process_xl(w, v);
}
void lsm6dsv80x_fifo_process_gyro(lsm6dsv80x_fifo_out_raw_t &w, Vector3<float> &v)
{
    if (driver_to_mdps)
        impl_fifo_process_gyro(w, v);
}
void lsm6dsv80x_fifo_process_gravity(lsm6dsv80x_fifo_out_raw_t &w, Vector3<float> &v)
{
    impl_fifo_process_gravity(w, v);
}
void lsm6dsv80x_fifo_process_timestamp(lsm6dsv80x_fifo_out_raw_t &w, float &v)
{
    impl_fifo_process_timestamp(w, v);
}
void lsm6dsv80x_fifo_process_temperature(lsm6dsv80x_fifo_out_raw_t &w, float &v)
{
    impl_fifo_process_temperature(w, v);
}
void lsm6dsv80x_fifo_process_sflp_game_rotation(lsm6dsv80x_fifo_out_raw_t &a,
                                                lsm6dsv80x_fifo_out_raw_t &b, Quaternion<float> &q)
{
    impl_fifo_process_sflp_game_rotation(a, b, q);
}
void lsm6dsv80x_read_sources(lsm6dsv80x_sources_t *s)
{
    if (!s)
        return;
    imu::Sources n{s->tap_x,      s->tap_y,    s->tap_z,       s->tap_single,
                   s->tap_double, s->tap_sign, s->sleep_change};
    impl_read_sources(&n);
    *s = {n.tap_x, n.tap_y, n.tap_z, n.tap_single, n.tap_double, n.tap_sign, n.sleep_change};
}
void lsm6dsv80x_sleep() { imu::sleep(); }
void lsm6dsv80x_set_debug(bool v) { imu::set_debug(v); }
