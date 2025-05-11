#include <stdio.h>
#include <unistd.h>
#include "lsm6dsv32x.h"
#include "lsm6dsv32x_reg.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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

#define I2C_TIMEOUT -1
#define TAG "LSM6DSV32X"

// #define USE_GBIAS 1

typedef float_t (*lsm6dsv32x_to_mg_t)(int16_t);
typedef float_t (*lsm6dsv32x_to_mdps_t)(int16_t);

static i2c_master_dev_handle_t dev_handle_LSM6DSV32;

stmdev_ctx_t dev_ctx = {0};

static lsm6dsv32x_to_mg_t lsm6dsv32x_to_mg;
static lsm6dsv32x_to_mdps_t lsm6dsv32x_to_mdps;
static lsm6dsv32x_cfg_t cfg = {
    .sampleRate = 120,
    .accelRange = 4,
    .gyroRange = 2000,
    .batching = LSM6DSV32X_XL_BATCHED_AT_120Hz,
    .fifoWatermark = 32,
    .fifoMode = LSM6DSV32X_STREAM_MODE,
    .timeout = 0};
static int max_elements_per_sample = 0;
static bool debug = false;

static int acc_range_to_lsm6_range(uint8_t range)
{
    switch (range)
    {
    default:
    case 4:
        lsm6dsv32x_to_mg = lsm6dsv32x_from_fs4_to_mg;
        return LSM6DSV32X_4g;
    case 8:
        lsm6dsv32x_to_mg = lsm6dsv32x_from_fs8_to_mg;
        return LSM6DSV32X_8g;
    case 16:
        lsm6dsv32x_to_mg = lsm6dsv32x_from_fs16_to_mg;
        return LSM6DSV32X_16g;
    case 32:
        lsm6dsv32x_to_mg = lsm6dsv32x_from_fs32_to_mg;
        return LSM6DSV32X_32g;
    }
}

static int gyro_range_to_lsm6_range(uint16_t range)
{
    switch (range)
    {
    case 125:
        lsm6dsv32x_to_mdps = lsm6dsv32x_from_fs125_to_mdps;
        return LSM6DSV32X_125dps;
    case 250:
        lsm6dsv32x_to_mdps = lsm6dsv32x_from_fs250_to_mdps;
        return LSM6DSV32X_250dps;
    case 500:
        lsm6dsv32x_to_mdps = lsm6dsv32x_from_fs500_to_mdps;
        return LSM6DSV32X_500dps;
    case 1000:
        lsm6dsv32x_to_mdps = lsm6dsv32x_from_fs1000_to_mdps;
        return LSM6DSV32X_1000dps;
    default:
    case 2000:
        lsm6dsv32x_to_mdps = lsm6dsv32x_from_fs2000_to_mdps;
        return LSM6DSV32X_2000dps;
    case 4000:
        lsm6dsv32x_to_mdps = lsm6dsv32x_from_fs4000_to_mdps;
        return LSM6DSV32X_4000dps;
    }
}

static int sampling_rate_to_lsm6_rate(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return LSM6DSV32X_ODR_AT_60Hz;
    case 120:
        return LSM6DSV32X_ODR_AT_120Hz;
    case 240:
        return LSM6DSV32X_ODR_AT_240Hz;
    case 480:
        return LSM6DSV32X_ODR_AT_480Hz;
    case 960:
        return LSM6DSV32X_ODR_AT_960Hz;
    case 1920:
        return LSM6DSV32X_ODR_AT_1920Hz;
    case 3840:
        return LSM6DSV32X_ODR_AT_3840Hz;
    case 7680:
        return LSM6DSV32X_ODR_AT_7680Hz;
    default:
        return LSM6DSV32X_ODR_AT_480Hz;
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

uint32_t lsm6dsv32x_from_f16_to_f32(uint16_t h)
{
    // Extract the sign, exponent, and mantissa from the 16-bit input
    uint16_t sign = (h >> 15) & 0x1;
    uint16_t exponent = (h >> 10) & 0x1F;
    uint16_t mantissa = h & 0x3FF;

    // Initialize a 32-bit floating point variable
    uint32_t result = 0;

    if (exponent == 0)
    {
        if (mantissa == 0)
        {
            // Zero
            result = sign << 31; // Just keep the sign bit
        }
        else
        {
            // Subnormal numbers
            float_t subnormal = (float_t)(mantissa) / (1 << 10);
            result = ((uint32_t)sign << 31) | (*(uint32_t *)&subnormal);
        }
    }
    else if (exponent == 31)
    {
        // Infinite or NaN
        result = (sign << 31) | (0xFF << 23) | (mantissa << 13);
    }
    else
    {
        // Normalized numbers
        uint32_t new_exp = exponent - 15 + 127; // Adjust the exponent
        result = (sign << 31) | (new_exp << 23) | (mantissa << 13);
    }

    return result;
}

static float_t npy_half_to_float(uint16_t h)
{
    union
    {
        float_t ret;
        uint32_t retbits;
    } conv;
    conv.retbits = lsm6dsv32x_from_f16_to_f32(h);
    return conv.ret;
}

static void sflp2q(float_t quat[4], uint16_t sflp[3])
{
    float_t sumsq = 0;

    quat[0] = npy_half_to_float(sflp[0]);
    quat[1] = npy_half_to_float(sflp[1]);
    quat[2] = npy_half_to_float(sflp[2]);

    for (uint8_t i = 0; i < 3; i++)
        sumsq += quat[i] * quat[i];

    if (sumsq > 1.0f)
        sumsq = 1.0f;

    quat[3] = sqrtf(1.0f - sumsq);
}

// Please note that is MANDATORY: return 0 -> no Error.
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t data_wr[4 + 1];
    data_wr[0] = reg;
    for (int i = 0; i < len; i++)
        data_wr[i + 1] = bufp[i];
    esp_err_t err = i2c_master_transmit(dev_handle_LSM6DSV32, data_wr, len + 1, I2C_TIMEOUT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write error to reg %02X, %s\n", reg, esp_err_to_name(err));
        return -1;
    }
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    esp_err_t err = i2c_master_transmit_receive(dev_handle_LSM6DSV32, &reg, 1, bufp, len, I2C_TIMEOUT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read error from reg %02X, %s\n", reg, esp_err_to_name(err));
        return -1;
    }
    return 0;
}

// Optional (may be required by driver)
static void platform_delay(uint32_t millisec)
{
    vTaskDelay(millisec / portTICK_PERIOD_MS);
}

static void setup_sampling_rate()
{
    lsm6dsv32x_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
    lsm6dsv32x_xl_full_scale_set(&dev_ctx, acc_range_to_lsm6_range(cfg.accelRange));
    lsm6dsv32x_gy_full_scale_set(&dev_ctx, gyro_range_to_lsm6_range(cfg.gyroRange));

    lsm6dsv32x_xl_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    lsm6dsv32x_gy_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    lsm6dsv32x_sflp_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    lsm6dsv32x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv32x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
}

static void setup_fifo()
{
    lsm6dsv32x_fifo_mode_set(&dev_ctx, LSM6DSV32X_BYPASS_MODE);
    lsm6dsv32x_fifo_watermark_set(&dev_ctx, cfg.fifoWatermark);
    lsm6dsv32x_fifo_xl_batch_set(&dev_ctx, cfg.batching);
    max_elements_per_sample = 1;
    lsm6dsv32x_fifo_gy_batch_set(&dev_ctx, cfg.batching);
    max_elements_per_sample++;
    lsm6dsv32x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV32X_TMSTMP_DEC_1);
    max_elements_per_sample++;
    lsm6dsv32x_fifo_sflp_raw_t fifo_sflp = {0};
    fifo_sflp.game_rotation = 1;
    max_elements_per_sample++;
    fifo_sflp.gravity = 1;
    max_elements_per_sample++;
#ifdef USE_GBIAS
    fifo_sflp.gbias = 1;
    max_elements_per_sample++;
#endif
    lsm6dsv32x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp); // Quaternion
    lsm6dsv32x_fifo_mode_set(&dev_ctx, cfg.fifoMode);
}

static void clear_fifo()
{
    lsm6dsv32x_fifo_mode_set(&dev_ctx, LSM6DSV32X_BYPASS_MODE);
    lsm6dsv32x_fifo_mode_set(&dev_ctx, LSM6DSV32X_STREAM_MODE);
}
static void setup_tapping()
{
    lsm6dsv32x_interrupt_mode_t irq = {0};
    lsm6dsv32x_tap_detection_t tap = {0};
    lsm6dsv32x_tap_thresholds_t tap_ths = {0};
    lsm6dsv32x_tap_time_windows_t tap_win = {0};
    irq.enable = 1;
    irq.lir = 0;
    lsm6dsv32x_interrupt_enable_set(&dev_ctx, irq);
    tap.tap_z_en = 1;
    tap.tap_x_en = 1;
    tap.tap_y_en = 0;
    lsm6dsv32x_tap_detection_set(&dev_ctx, tap);
    tap_ths.z = 0x1f;
    tap_ths.x = 0x1f;
    tap_ths.y = 0x1f;
    lsm6dsv32x_tap_thresholds_set(&dev_ctx, tap_ths);
    tap_win.tap_gap = 3;
    tap_win.shock = 2;
    tap_win.quiet = 3;
    lsm6dsv32x_tap_time_windows_set(&dev_ctx, tap_win);
    lsm6dsv32x_tap_axis_priority_set(&dev_ctx, LSM6DSV32X_XZY);

    lsm6dsv32x_tap_mode_set(&dev_ctx, LSM6DSV32X_BOTH_SINGLE_DOUBLE);
}

static void setup_filter()
{
    lsm6dsv32x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV32X_GY_XTREME);
    lsm6dsv32x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv32x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV32X_XL_XTREME);
    lsm6dsv32x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);

    // lsm6dsv32x_filt_xl_hp_set(&dev_ctx, PROPERTY_ENABLE);

    // lsm6dsv32x_filt_xl_fast_settling_set(&dev_ctx, PROPERTY_ENABLE);

    // lsm6dsv32x_filt_xl_hp_mode_set(&dev_ctx, LSM6DSV32X_HP_MD_NORMAL);

    // lsm6dsv32x_filt_sixd_feed_set(&dev_ctx, PROPERTY_DISABLE);

    // lsm6dsv32x_filt_gy_eis_lp_bandwidth_set(&dev_ctx, LSM6DSV32X_EIS_LP_NORMAL);
}
void lsm6dsv32x_read_sources(lsm6dsv32x_sources_t *sources)
{
    lsm6dsv32x_all_sources_t status;
    lsm6dsv32x_all_sources_get(&dev_ctx, &status);

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
    if (status.drdy_ah_qvar)
    {
        printf("QVAR ready\n");
    }
    sources->tap_sign = status.tap_sign;
}

/// @brief Check if fifo contains data
/// @return Number of samples or -1 if error. NOTE: 0 means no data.
int lsm6dsv32x_fifo_data_available()
{
    lsm6dsv32x_fifo_status_t fifo_status;
    if (-1 == lsm6dsv32x_fifo_status_get(&dev_ctx, &fifo_status))
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

int lsm6dsv32x_read_fifo_element(fifo_element_t *el)
{
    float_t quat[4];
    int16_t *axis;
    int16_t *datax;
    int16_t *datay;
    int16_t *dataz;
    int32_t *ts;
    int32_t timestamp = 0;

    // IMPORTANT BEGIN
    // This number must be kept in sync with the number of variables
    // configured in the setup_fifo() function.
    int elements_per_sample = 0;
    // IMPORTANT END

    // static int samples_per_sec = 0;

    while (elements_per_sample != max_elements_per_sample)
    {
        lsm6dsv32x_fifo_out_raw_t f_data;

        if (-1 == lsm6dsv32x_fifo_out_raw_get(&dev_ctx, &f_data))
        {
            ESP_LOGE(TAG, "FIFO read error\n");
            return -1;
        }

        if (f_data.tag == LSM6DSV32X_FIFO_EMPTY)
            return 0;

        datax = (int16_t *)&f_data.data[0];
        datay = (int16_t *)&f_data.data[2];
        dataz = (int16_t *)&f_data.data[4];
        ts = (int32_t *)&f_data.data[0];

        switch (f_data.tag)
        {
        case LSM6DSV32X_XL_NC_TAG:
            el->acc.x = lsm6dsv32x_to_mg(*datax) * 0.001;
            el->acc.y = lsm6dsv32x_to_mg(*datay) * 0.001;
            el->acc.z = lsm6dsv32x_to_mg(*dataz) * 0.001;
            elements_per_sample++;
            break;
        case LSM6DSV32X_GY_NC_TAG:
            el->gyro.x = lsm6dsv32x_to_mdps(*datax) * 0.001;
            el->gyro.y = lsm6dsv32x_to_mdps(*datay) * 0.001;
            el->gyro.z = lsm6dsv32x_to_mdps(*dataz) * 0.001;
            elements_per_sample++;
            break;
        case LSM6DSV32X_TIMESTAMP_TAG:
            timestamp = *ts;
            el->timestamp = timestamp * 0.00002175;
            elements_per_sample++;
            break;
#ifdef USE_GBIAS
        case LSM6DSV32X_SFLP_GYROSCOPE_BIAS_TAG:
            axis = (int16_t *)&f_data.data[0];
            el->gbias.x = lsm6dsv32x_from_fs125_to_mdps(axis[0]);
            el->gbias.y = lsm6dsv32x_from_fs125_to_mdps(axis[1]);
            el->gbias.z = lsm6dsv32x_from_fs125_to_mdps(axis[2]);
            elements_per_sample++;
            break;
#endif
        case LSM6DSV32X_SFLP_GRAVITY_VECTOR_TAG:
            axis = (int16_t *)&f_data.data[0];
            el->gravity.x = lsm6dsv32x_from_sflp_to_mg(axis[0]) * 0.001;
            el->gravity.y = lsm6dsv32x_from_sflp_to_mg(axis[1]) * 0.001;
            el->gravity.z = lsm6dsv32x_from_sflp_to_mg(axis[2]) * 0.001;
            elements_per_sample++;
            break;
        case LSM6DSV32X_SFLP_GAME_ROTATION_VECTOR_TAG:
            sflp2q(quat, (uint16_t *)&f_data.data[0]);
            el->q.w = quat[3];
            el->q.x = quat[0];
            el->q.y = quat[1];
            el->q.z = quat[2];
            elements_per_sample++;
            break;
        default:
            ESP_LOGW(TAG, "Unknown tag %d\n", f_data.tag);
            break;
        }
    }
    if (debug)
    {
        RUN_CODE_EVERY_START(1.0);
        printf("%.2f: FIFO acc: %.3f %.3f %.3f, gyro: %.3f %.3f %.3f, gravity: %.3f %.3f %.3f, q: %.3f %.3f %.3f %.3f\n",
               el->timestamp,
               el->acc.x, el->acc.y, el->acc.z,
               el->gyro.x, el->gyro.y, el->gyro.z,
               el->gravity.x, el->gravity.y, el->gravity.z,
               el->q.w, el->q.x, el->q.y, el->q.z);
        RUN_CODE_EVERY_END();
    }
    // static int batches_per_sec = 0;
    // batches_per_sec++;
    // RUN_CODE_EVERY_START(1.0)
    //     printf("%d s/sec, %d bt/sec\n", samples_per_sec, batches_per_sec);
    //     samples_per_sec = 0;
    //     batches_per_sec = 0;
    // RUN_CODE_EVERY_END()
    int ret = elements_per_sample == max_elements_per_sample;
    if (ret != 1)
        ESP_LOGW(TAG, "FIFO element incomplete, %d\n", elements_per_sample);
    return ret;
}

// float lsm6dsv32x_get_timestamp_resolution(float sample_rate)
// {
//     int8_t freq_fine;
//     if (-1 == lsm6dsv32x_odr_cal_reg_get(&dev_ctx, &freq_fine))
//     {
//         printf("!!! Error getting odr calibration register !!!\n");
//         return 0;
//     }
//     return 7680.0 * (1 + 0.0013 * (float)freq_fine) / sampling_rate_to_odrcoeff(sample_rate);
// }

void lsm6dsv32x_init_i2c(i2c_master_dev_handle_t dev_handle)
{
    dev_handle_LSM6DSV32 = dev_handle;
}

void lsm6dsv32x_start_sampling(bool start)
{
    printf("##### LSM6DSV32X start sampling, %d #####\n", start);

    if (start)
    {
        clear_fifo();

        int8_t freq_fine;
        lsm6dsv32x_odr_cal_reg_get(&dev_ctx, &freq_fine);
        float tactual = 1 / (46080.0 * (1 + 0.0013 * (float)freq_fine));
        float odractual = 7680.0 * (1 + 0.0013 * (float)freq_fine) / sampling_rate_to_odrcoeff(cfg.sampleRate);
        ESP_LOGI(TAG, "LSM6DSV32X started, sample rate: %d, acc range: %d, gyro range: %d timestamp res %fs, odr actual %fHz", cfg.sampleRate, cfg.accelRange, cfg.gyroRange, tactual, odractual);
    }
}

void lsm6dsv32x_start_tapping(bool start)
{
    // printf("##### LSM6DSV32X start tapping, %d #####\n", start);
    // setup_sampling_rate();
    if (start)
        setup_tapping();
    uint8_t property_enable = start ? PROPERTY_ENABLE : PROPERTY_DISABLE;
    lsm6dsv32x_pin_int_route_t pin_int2 = {0};
    pin_int2.double_tap = property_enable;
    pin_int2.single_tap = property_enable;
    // pin_int2.sleep_change = property_enable;
    lsm6dsv32x_pin_int2_route_set(&dev_ctx, &pin_int2);
}

static void lsm6dsv32x_reset()
{
    lsm6dsv32x_reset_t rst;

    if (-1 == lsm6dsv32x_reset_set(&dev_ctx, LSM6DSV32X_RESTORE_CTRL_REGS | LSM6DSV32X_GLOBAL_RST))
        ESP_LOGE(TAG, "LSM6DSV32X reset failed\n");

    int rst_cnt = 0;
    const int sleep_time = 5000;
    int total_sleep_time = 0;
    do
    {
        usleep(sleep_time);
        lsm6dsv32x_reset_get(&dev_ctx, &rst);
        rst_cnt++;
        total_sleep_time += sleep_time;
    } while (rst != LSM6DSV32X_READY && total_sleep_time < 1000000);
    if (rst != LSM6DSV32X_READY)
        ESP_LOGE(TAG, "LSM6DSV32X reset failed\n");
}

void lsm6dsv32x_config(lsm6dsv32x_cfg_t *newCfg)
{
    cfg = *newCfg;
    if (debug)
    {
        // printf("##### LSM6DSV32X config #####\n");
        printf("LSM6DSV32X config:\n");
        printf("  sample rate: %d\n", cfg.sampleRate);
        printf("  accel range: %d\n", cfg.accelRange);
        printf("  gyro range: %d\n", cfg.gyroRange);
        printf("  batching: %d\n", cfg.batching);
        printf("  fifo watermark: %d\n", cfg.fifoWatermark);
        printf("  fifo mode: %d\n", cfg.fifoMode);
        printf("  timeout: %ld\n", cfg.timeout);
    }
    lsm6dsv32x_reset();
    setup_sampling_rate();
    setup_fifo();
    // setup_filter();
}

void lsm6dsv32x_set_debug(bool d)
{
    debug = d;
}

void lsm6dsv32x_init()
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &dev_handle_LSM6DSV32;

    uint8_t whoamI = 0;
    if (-1 == lsm6dsv32x_device_id_get(&dev_ctx, &whoamI))
        ESP_LOGE(TAG, "Who am I get failed\n");

    if (whoamI != LSM6DSV32X_ID)
        ESP_LOGE(TAG, "Who am I doesn't match\n");
}
