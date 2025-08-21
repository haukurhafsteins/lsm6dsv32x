#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "lsm6dsv80x.hpp"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_timer.h"
#include "vectors.h"

#define PRINT_FLOAT(name, value, period)         \
    {                                            \
        static float last = 0;                   \
        float now = esp_timer_get_time() * 1e-6; \
        if (now - last > period)                 \
        {                                        \
            last = now;                          \
            printf(name, value);                 \
        }                                        \
    }

#define get_time_in_seconds() (esp_timer_get_time() * 1e-6)

#define RUN_CODE_EVERY_START(period)       \
    {                                      \
        static float last = 0;             \
        float now = get_time_in_seconds(); \
        if (now - last > period)           \
        {                                  \
            last = now;

#define RUN_CODE_EVERY_END() \
    }                        \
    }

#define TAG "LSM6DSV80X"

// #define USE_GBIAS 1

typedef float_t (*lsm6dsv80x_to_mg_t)(int16_t);
typedef float_t (*lsm6dsv80x_to_mdps_t)(int16_t);

static spi_device_handle_t dev_handle_LSM6DSV32;

stmdev_ctx_t dev_ctx = {};

static lsm6dsv80x_to_mg_t lsm6dsv80x_to_mg;
static lsm6dsv80x_to_mdps_t lsm6dsv80x_to_mdps;
static lsm6dsv80x_cfg_t cfg = {
    .sampleRate = 120,
    .accelRange = 4,
    .gyroRange = 2000,
    .batching = LSM6DSV80X_XL_BATCHED_AT_120Hz,
    .fifoWatermark = 32,
    .fifoMode = LSM6DSV80X_STREAM_MODE,
    .timeout = 0};
static bool debug = false;

static lsm6dsv80x_xl_full_scale_t acc_range_to_lsm6_range(uint8_t range)
{
    switch (range)
    {
    default:
    case 2:
        lsm6dsv80x_to_mg = lsm6dsv80x_from_fs2_to_mg;
        return LSM6DSV80X_2g;
    case 4:
        lsm6dsv80x_to_mg = lsm6dsv80x_from_fs4_to_mg;
        return LSM6DSV80X_4g;
    case 8:
        lsm6dsv80x_to_mg = lsm6dsv80x_from_fs8_to_mg;
        return LSM6DSV80X_8g;
    case 16:
        lsm6dsv80x_to_mg = lsm6dsv80x_from_fs16_to_mg;
        return LSM6DSV80X_16g;
    }
}

static lsm6dsv80x_gy_full_scale_t gyro_range_to_lsm6_range(uint16_t range)
{
    switch (range)
    {
    case 250:
        lsm6dsv80x_to_mdps = lsm6dsv80x_from_fs250_to_mdps;
        return LSM6DSV80X_250dps;
    case 500:
        lsm6dsv80x_to_mdps = lsm6dsv80x_from_fs500_to_mdps;
        return LSM6DSV80X_500dps;
    case 1000:
        lsm6dsv80x_to_mdps = lsm6dsv80x_from_fs1000_to_mdps;
        return LSM6DSV80X_1000dps;
    default:
    case 2000:
        lsm6dsv80x_to_mdps = lsm6dsv80x_from_fs2000_to_mdps;
        return LSM6DSV80X_2000dps;
    case 4000:
        lsm6dsv80x_to_mdps = lsm6dsv80x_from_fs4000_to_mdps;
        return LSM6DSV80X_4000dps;
    }
}

static lsm6dsv80x_data_rate_t sampling_rate_to_lsm6_rate(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return LSM6DSV80X_ODR_AT_60Hz;
    case 120:
        return LSM6DSV80X_ODR_AT_120Hz;
    case 240:
        return LSM6DSV80X_ODR_AT_240Hz;
    case 480:
        return LSM6DSV80X_ODR_AT_480Hz;
    case 960:
        return LSM6DSV80X_ODR_AT_960Hz;
    case 1920:
        return LSM6DSV80X_ODR_AT_1920Hz;
    case 3840:
        return LSM6DSV80X_ODR_AT_3840Hz;
    case 7680:
        return LSM6DSV80X_ODR_AT_7680Hz;
    default:
        return LSM6DSV80X_ODR_AT_480Hz;
    }
}

static lsm6dsv80x_sflp_data_rate_t sampling_rate_to_sflp_rate(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return LSM6DSV80X_SFLP_60Hz;
    case 120:
        return LSM6DSV80X_SFLP_120Hz;
    case 240:
        return LSM6DSV80X_SFLP_240Hz;
    case 480:
    case 960:
    case 1920:
    case 3840:
    case 7680:
    default:
        return LSM6DSV80X_SFLP_480Hz;
    }
}

static int sampling_rate_to_odrcoeff(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return 128;
    case 120:
        return 64;
    case 240:
        return 32;
    case 480:
        return 16;
    case 960:
        return 8;
    case 1920:
        return 4;
    case 3840:
        return 2;
    case 7680:
        return 1;
    default:
        return 16;
    }
}

static float_t npy_half_to_float(uint16_t h)
{
    union
    {
        float_t ret;
        uint32_t retbits;
    } conv;
    conv.retbits = lsm6dsv80x_from_f16_to_f32(h);
    return conv.ret;
}

static void sflp2q(Quaternion<float> &quat, uint16_t sflp[3])
{
    float sumsq = 0.0f;
    quat.x = npy_half_to_float(sflp[0]);
    quat.y = npy_half_to_float(sflp[1]);
    quat.z = npy_half_to_float(sflp[2]);
    quat.w = 0.0f; // w is not provided in SFLP, we reconstruct it

    sumsq = quat.x * quat.x + quat.y * quat.y + quat.z * quat.z;

    if (sumsq > 1.0f) sumsq = 1.0f;
    if (sumsq < 0.0f) sumsq = 0.0f;

    quat.w = sqrtf(1.0f - sumsq); // positive w
    quat.normalize();
}

// Assumes Quaternion<T> has public members: x, y, z, w  (xyzw order).
// Reconstructs w from x,y,z and optionally matches hemisphere vs q_prev.
// If q_prev == nullptr, it just returns a unit quaternion with w >= 0.
template <typename T>
inline void sflp2q_continuous(Quaternion<T>& q,
                              const Quaternion<T>* q_prev,   // pass previous quaternion, or nullptr on first call
                              const uint16_t sflp[3])        // SFLP half-precision components for x,y,z
{
    // Load x,y,z from your half->float converter, then cast to T.
    q.x = static_cast<T>(npy_half_to_float(sflp[0]));
    q.y = static_cast<T>(npy_half_to_float(sflp[1]));
    q.z = static_cast<T>(npy_half_to_float(sflp[2]));

    // x^2 + y^2 + z^2 with clamping for numeric safety
    T sumsq = q.x*q.x + q.y*q.y + q.z*q.z;
    if (sumsq > static_cast<T>(1)) sumsq = static_cast<T>(1);
    if (sumsq < static_cast<T>(0)) sumsq = static_cast<T>(0);

    // Reconstruct positive w
    q.w = static_cast<T>(std::sqrt(static_cast<long double>(1) - static_cast<long double>(sumsq)));

    // (Optional) light renormalization to kill drift
    {
        T n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (n > static_cast<T>(0)) {
            T invn = static_cast<T>(1) / static_cast<T>(std::sqrt(static_cast<long double>(n)));
            q.x *= invn; q.y *= invn; q.z *= invn; q.w *= invn;
        }
    }

    // Hemisphere continuity: ensure dot(q, q_prev) >= 0 by flipping all components if needed
    if (q_prev) {
        T dot = q.x*q_prev->x + q.y*q_prev->y + q.z*q_prev->z + q.w*q_prev->w;
        if (dot < static_cast<T>(0)) {
            q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
        }
    }
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t tx_buf[1 + len];
    tx_buf[0] = reg & 0x7F; // Write command
    memcpy(&tx_buf[1], bufp, len);

    spi_transaction_t t = {};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = NULL;

    esp_err_t err = spi_device_polling_transmit(dev_handle_LSM6DSV32, &t);
    if (err != ESP_OK)
    {
        printf("SPI write error: %s\n", esp_err_to_name(err));
        return -1;
    }
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t tx_buf[1 + len];
    uint8_t rx_buf[1 + len]; // First byte will be dummy

    tx_buf[0] = reg | 0x80;        // Read command
    memset(&tx_buf[1], 0xFF, len); // Dummy bytes

    spi_transaction_t t = {};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t err = spi_device_polling_transmit(dev_handle_LSM6DSV32, &t);
    if (err != ESP_OK)
    {
        printf("SPI read error: %s\n", esp_err_to_name(err));
        return -1;
    }

    memcpy(bufp, &rx_buf[1], len); // Skip dummy byte
    return 0;
}

// Optional (may be required by driver)
static void platform_delay(uint32_t millisec)
{
    vTaskDelay(millisec / portTICK_PERIOD_MS);
}

static void setup_sampling_rate()
{
    // lsm6dsv80x_xl_offset_on_out_set(&dev_ctx, PROPERTY_DISABLE);
    // lsm6dsv80x_xl_offset_mg_set(&dev_ctx, {0, 0, 0});
    // lsm6dsv80x_hg_xl_offset_mg_set(&dev_ctx, {0, 0, 0});
    // lsm6dsv80x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV80X_HG_XL_ODR_AT_120Hz);

    lsm6dsv80x_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
    lsm6dsv80x_xl_full_scale_set(&dev_ctx, acc_range_to_lsm6_range(cfg.accelRange));
    lsm6dsv80x_gy_full_scale_set(&dev_ctx, gyro_range_to_lsm6_range(cfg.gyroRange));

    lsm6dsv80x_xl_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    lsm6dsv80x_gy_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    lsm6dsv80x_sflp_data_rate_set(&dev_ctx, sampling_rate_to_sflp_rate(cfg.sampleRate));

    lsm6dsv80x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv80x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
}

static void setup_fifo()
{
    // lsm6dsv80x_4d_mode_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv80x_fifo_mode_set(&dev_ctx, LSM6DSV80X_BYPASS_MODE);
    lsm6dsv80x_fifo_watermark_set(&dev_ctx, cfg.fifoWatermark);
    lsm6dsv80x_fifo_xl_batch_set(&dev_ctx, cfg.batching);
    lsm6dsv80x_fifo_gy_batch_set(&dev_ctx, (lsm6dsv80x_fifo_gy_batch_t)cfg.batching);
    lsm6dsv80x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV80X_TMSTMP_DEC_1);
    lsm6dsv80x_fifo_sflp_raw_t fifo_sflp = {};
    fifo_sflp.game_rotation = 1;
    fifo_sflp.gravity = 1;
#ifdef USE_GBIAS
    fifo_sflp.gbias = 1;
#endif
    lsm6dsv80x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);
    lsm6dsv80x_fifo_mode_set(&dev_ctx, LSM6DSV80X_STREAM_MODE);
}

static void clear_fifo()
{
    lsm6dsv80x_fifo_mode_set(&dev_ctx, LSM6DSV80X_BYPASS_MODE);
    lsm6dsv80x_fifo_mode_set(&dev_ctx, LSM6DSV80X_STREAM_MODE);
}
static void setup_tapping()
{
    lsm6dsv80x_interrupt_mode_t irq = {};
    lsm6dsv80x_tap_detection_t tap = {};
    lsm6dsv80x_tap_thresholds_t tap_ths = {};
    lsm6dsv80x_tap_time_windows_t tap_win = {};
    irq.enable = 1;
    irq.lir = 0;
    lsm6dsv80x_interrupt_enable_set(&dev_ctx, irq);
    tap.tap_z_en = 1;
    tap.tap_x_en = 1;
    tap.tap_y_en = 0;
    lsm6dsv80x_tap_detection_set(&dev_ctx, tap);
    tap_ths.z = 0x1f;
    tap_ths.x = 0x1f;
    tap_ths.y = 0x1f;
    lsm6dsv80x_tap_thresholds_set(&dev_ctx, tap_ths);
    tap_win.tap_gap = 3;
    tap_win.shock = 2;
    tap_win.quiet = 3;
    lsm6dsv80x_tap_time_windows_set(&dev_ctx, tap_win);
    lsm6dsv80x_tap_axis_priority_set(&dev_ctx, LSM6DSV80X_XZY);

    lsm6dsv80x_tap_mode_set(&dev_ctx, LSM6DSV80X_BOTH_SINGLE_DOUBLE);
}

static void setup_filter()
{
    lsm6dsv80x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV80X_GY_XTREME);
    lsm6dsv80x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv80x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV80X_XL_XTREME);
    lsm6dsv80x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);

    // lsm6dsv80x_filt_xl_hp_set(&dev_ctx, PROPERTY_ENABLE);

    // lsm6dsv80x_filt_xl_fast_settling_set(&dev_ctx, PROPERTY_ENABLE);

    // lsm6dsv80x_filt_xl_hp_mode_set(&dev_ctx, LSM6DSV80X_HP_MD_NORMAL);

    // lsm6dsv80x_filt_sixd_feed_set(&dev_ctx, PROPERTY_DISABLE);

    // lsm6dsv80x_filt_gy_eis_lp_bandwidth_set(&dev_ctx, LSM6DSV80X_EIS_LP_NORMAL);
}
void lsm6dsv80x_read_sources(lsm6dsv80x_sources_t *sources)
{
    lsm6dsv80x_all_sources_t status;
    lsm6dsv80x_all_sources_get(&dev_ctx, &status);

    if (status.single_tap)
    {
        sources->tap_single = true;
        // printf("Single tap detected, sign %d, x %d, y %d, z %d\n", status.tap_sign, status.tap_x, status.tap_y, status.tap_z);
    }
    if (status.double_tap)
    {
        sources->tap_double = true;
        // printf("Double tap detected, sign %d, x %d, y %d, z %d\n", status.tap_sign, status.tap_x, status.tap_y, status.tap_z);
    }
    if (status.tap_x)
    {
        sources->tap_x = true;
        // printf("Tap on X\n");
    }
    if (status.tap_y)
    {
        sources->tap_y = true;
        // printf("Tap on Y\n");
    }
    if (status.tap_z)
    {
        sources->tap_z = true;
        // printf("Tap on Z\n");
    }
    if (status.sleep_change)
    {
        sources->sleep_change = true;
        printf("Sleep change\n");
    }
    sources->tap_sign = status.tap_sign;
}

/// @brief Check if fifo contains data
/// @return Number of samples or -1 if error. NOTE: 0 means no data.
int lsm6dsv80x_fifo_data_available()
{
    lsm6dsv80x_fifo_status_t fifo_status;
    if (-1 == lsm6dsv80x_fifo_status_get(&dev_ctx, &fifo_status))
    {
        ESP_LOGE(TAG, "FIFO status read error");
        return -1;
    }
    if (fifo_status.fifo_ovr == 1)
    {
        ESP_LOGE(TAG, "FIFO overrun");
        clear_fifo();
        return -1;
    }
    return fifo_status.fifo_level;
}

int lsm6dsv80x_fifo_read_element(lsm6dsv80x_fifo_out_raw_t &f_data)
{
    return lsm6dsv80x_fifo_out_raw_get(&dev_ctx, &f_data);
}

void lsm6dsv80x_fifo_process_xl(lsm6dsv80x_fifo_out_raw_t &f_data, Vector3<float> &acc)
{
    int16_t *datax = (int16_t *)&f_data.data[0];
    int16_t *datay = (int16_t *)&f_data.data[2];
    int16_t *dataz = (int16_t *)&f_data.data[4];

    acc.x = lsm6dsv80x_to_mg(*datax) * 0.001;
    acc.y = lsm6dsv80x_to_mg(*datay) * 0.001;
    acc.z = lsm6dsv80x_to_mg(*dataz) * 0.001;
}

void lsm6dsv80x_fifo_process_gyro(lsm6dsv80x_fifo_out_raw_t &f_data, Vector3<float> &gyro)
{
    int16_t *datax = (int16_t *)&f_data.data[0];
    int16_t *datay = (int16_t *)&f_data.data[2];
    int16_t *dataz = (int16_t *)&f_data.data[4];

    gyro.x = lsm6dsv80x_to_mdps(*datax) * 0.001;
    gyro.y = lsm6dsv80x_to_mdps(*datay) * 0.001;
    gyro.z = lsm6dsv80x_to_mdps(*dataz) * 0.001;
}

void lsm6dsv80x_fifo_process_timestamp(lsm6dsv80x_fifo_out_raw_t &f_data, float &timestamp)
{
    int32_t *ts = (int32_t *)f_data.data;
    timestamp = *ts * 0.0000217f; // Convert to seconds
}
void lsm6dsv80x_fifo_process_gravity(lsm6dsv80x_fifo_out_raw_t &f_data, Vector3<float> &gravity)
{
    int16_t *axis = (int16_t *)&f_data.data[0];
    gravity.x = lsm6dsv80x_from_sflp_to_mg(axis[0]) * 0.001;
    gravity.y = lsm6dsv80x_from_sflp_to_mg(axis[1]) * 0.001;
    gravity.z = lsm6dsv80x_from_sflp_to_mg(axis[2]) * 0.001;
}

static inline uint16_t u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// f_data1 and f_data2 must be the two consecutive 0x13 words for one sample
void lsm6dsv80x_fifo_process_sflp_game_rotation(lsm6dsv80x_fifo_out_raw_t &f_data1,
                                                lsm6dsv80x_fifo_out_raw_t &f_data2,
                                                Quaternion<float> &q)
{
    auto sensor_id = [](uint8_t tag){ return tag & 0x1F; };      // 5-bit sensor code
    auto tag_cnt   = [](uint8_t tag){ return (tag >> 6) & 0x03; }; // 2-bit rolling counter

    // Basic validation: both must be game-rotation vector (0x13)
    if (sensor_id(f_data1.tag) != 0x13 || sensor_id(f_data2.tag) != 0x13) {
        // Not our packets; zero out and return
        q = {0,0,0,1};
        return;
    }

    // Markers in X (little-endian): 0x0000 = first word, 0x0001 = second word
    uint16_t m1 = u16_le(&f_data1.data[0]);
    uint16_t m2 = u16_le(&f_data2.data[0]);

    // Optional sanity: they usually share the same TAG_CNT
    // (don’t hard-fail if they don’t; just a useful check)
    // uint8_t c1 = tag_cnt(f_data1.tag), c2 = tag_cnt(f_data2.tag);

    uint16_t hx=0, hy=0, hz=0, hw=0;

    if (m1 == 0x0000 && m2 == 0x0001) {
        // first then second (expected order)
        hx = u16_le(&f_data1.data[2]); // QUATX
        hy = u16_le(&f_data1.data[4]); // QUATY
        hz = u16_le(&f_data2.data[2]); // QUATZ
        hw = u16_le(&f_data2.data[4]); // QUATW
    } else if (m1 == 0x0001 && m2 == 0x0000) {
        // reversed arguments — still handle
        hx = u16_le(&f_data2.data[2]); // QUATX
        hy = u16_le(&f_data2.data[4]); // QUATY
        hz = u16_le(&f_data1.data[2]); // QUATZ
        hw = u16_le(&f_data1.data[4]); // QUATW
    } else {
        // Unexpected markers; bail safely
        q = {0,0,0,1};
        return;
    }

    // Convert half -> float (use your npy_half_to_float)
    q.x = npy_half_to_float(hx);
    q.y = npy_half_to_float(hy);
    q.z = npy_half_to_float(hz);
    q.w = npy_half_to_float(hw);

    // Guard + normalize
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
        q = {0,0,0,1};
        return;
    }
    float n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (n2 > 0.0f) {
        float inv = 1.0f / std::sqrt(n2);
        q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
    }
}

float lsm6dsv80x_get_timestamp_resolution(float sample_rate)
{
    int8_t freq_fine;
    if (-1 == lsm6dsv80x_odr_cal_reg_get(&dev_ctx, &freq_fine))
    {
        printf("!!! Error getting odr calibration register !!!\n");
        return 0;
    }
    return 7680.0 * (1 + 0.0013 * (float)freq_fine) / sampling_rate_to_odrcoeff(sample_rate);
}

void lsm6dsv80x_init_spi(spi_device_handle_t *dev_handle)
{
    dev_handle_LSM6DSV32 = *dev_handle;
}

void lsm6dsv80x_start_sampling(bool start)
{
    printf("##### LSM6DSV80X %s sampling #####\n", start ? "start" : "stop");

    if (start)
    {
        clear_fifo();

        int8_t freq_fine;
        lsm6dsv80x_odr_cal_reg_get(&dev_ctx, &freq_fine);
        float tactual = 1 / (46080.0 * (1 + 0.0013 * (float)freq_fine));
        float odractual = 7680.0 * (1 + 0.0013 * (float)freq_fine) / sampling_rate_to_odrcoeff(cfg.sampleRate);
        ESP_LOGI(TAG, "LSM6DSV80X started, sample rate: %d, acc range: %d, gyro range: %d timestamp res %fs, odr actual %fHz", cfg.sampleRate, cfg.accelRange, cfg.gyroRange, tactual, odractual);
    }
}

void lsm6dsv80x_start_tapping(bool start)
{
    // printf("##### LSM6DSV80X start tapping, %d #####\n", start);
    // setup_sampling_rate();
    if (start)
        setup_tapping();
    uint8_t property_enable = start ? PROPERTY_ENABLE : PROPERTY_DISABLE;
    lsm6dsv80x_pin_int_route_t pin_int2 = {};
    pin_int2.double_tap = property_enable;
    pin_int2.single_tap = property_enable;
    // pin_int2.sleep_change = property_enable;
    lsm6dsv80x_pin_int2_route_set(&dev_ctx, &pin_int2);
}

static void lsm6dsv80x_reset()
{
    lsm6dsv80x_reset_t rst;

    if (-1 == lsm6dsv80x_reset_set(&dev_ctx, (lsm6dsv80x_reset_t)(LSM6DSV80X_RESTORE_CTRL_REGS | LSM6DSV80X_GLOBAL_RST)))
        ESP_LOGE(TAG, "LSM6DSV80X reset failed\n");

    int rst_cnt = 0;
    const int sleep_time = 5000;
    int total_sleep_time = 0;
    do
    {
        usleep(sleep_time);
        lsm6dsv80x_reset_get(&dev_ctx, &rst);
        rst_cnt++;
        total_sleep_time += sleep_time;
    } while (rst != LSM6DSV80X_READY && total_sleep_time < 1000000);
    if (rst != LSM6DSV80X_READY)
        ESP_LOGE(TAG, "LSM6DSV80X reset failed\n");
}

void lsm6dsv80x_config(lsm6dsv80x_cfg_t *newCfg)
{
    cfg = *newCfg;
    if (debug)
    {
        // printf("##### LSM6DSV80X config #####\n");
        printf("LSM6DSV80X config:\n");
        printf("  sample rate: %d\n", cfg.sampleRate);
        printf("  accel range: %d\n", cfg.accelRange);
        printf("  gyro range: %d\n", cfg.gyroRange);
        printf("  batching: %d\n", cfg.batching);
        printf("  fifo watermark: %d\n", cfg.fifoWatermark);
        printf("  fifo mode: %d\n", cfg.fifoMode);
        printf("  timeout: %ld\n", cfg.timeout);
    }
    lsm6dsv80x_reset();
    setup_sampling_rate();
    setup_fifo();
    // setup_filter();
}

void lsm6dsv80x_set_debug(bool d)
{
    debug = d;
}

void lsm6dsv80x_init()
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &dev_handle_LSM6DSV32;
    uint8_t whoamI = 0;
    if (-1 == lsm6dsv80x_device_id_get(&dev_ctx, &whoamI))
        ESP_LOGE(TAG, "Who am I get failed\n");

    if (whoamI != 0x73)
        ESP_LOGE(TAG, "Who am I doesn't match\n");
}
