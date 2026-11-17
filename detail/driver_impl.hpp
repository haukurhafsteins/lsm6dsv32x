#include "../include/imu_lsm6dsv.hpp"
#include "register_api.hpp"
#include "rtos/Log.hpp"
#include "rtos/Spi.hpp"
#include "rtos/time.hpp"
#include "vectors.h"
#include <cstring>
#include <stdio.h>

#define TAG "LSM6DSV"

namespace
{
// #define USE_GBIAS 1

typedef float_t (*driver_to_mg_t)(int16_t);
typedef float_t (*driver_to_mdps_t)(int16_t);

// Transport is the platform-neutral rtos SPI device (command_bits = 8: the
// register byte travels in the command phase, the data phase stays clean).
static rtos::SpiDevice *s_spi = nullptr;

stmdev_ctx_t dev_ctx = {};

static driver_to_mg_t driver_to_mg;
static driver_to_mdps_t driver_to_mdps;
struct DriverConfig
{
    uint16_t sampleRate;
    uint8_t accelRange;
    uint16_t gyroRange;
    reg::fifo_xl_batch_t batching;
    uint16_t fifoWatermark;
    reg::fifo_mode_t fifoMode;
    uint32_t timeout;
    bool fifoTemperature = false;
};
static imu::InitStatus status = imu::InitStatus::NotInitialized;
static bool probing = false;
static bool ready() { return status == imu::InitStatus::Ready; }
static float impl_get_timestamp_resolution();
static DriverConfig cfg = {.sampleRate = 120,
                           .accelRange = 4,
                           .gyroRange = 2000,
                           .batching = reg::v_XL_BATCHED_AT_120Hz,
                           .fifoWatermark = 32,
                           .fifoMode = reg::v_STREAM_MODE,
                           .timeout = 0};
static bool debug = false;
static uint32_t overrun_count = 0;
static float timestamp_lsb_sec = 0.0000217f; // default from datasheet (21.7 us)

static reg::xl_full_scale_t acc_range_to_lsm6_range(uint8_t range)
{
    switch (range)
    {
    default:
#if IMU_LSM6DSV_PART == 80
    case 2:
        driver_to_mg = reg::from_fs2_to_mg;
        return reg::v_2g;
#endif
    case 4:
        driver_to_mg = reg::from_fs4_to_mg;
        return reg::v_4g;
    case 8:
        driver_to_mg = reg::from_fs8_to_mg;
        return reg::v_8g;
    case 16:
        driver_to_mg = reg::from_fs16_to_mg;
        return reg::v_16g;
#if IMU_LSM6DSV_PART == 32
    case 32:
        driver_to_mg = reg::from_fs32_to_mg;
        return reg::v_32g;
#endif
    }
}

static reg::gy_full_scale_t gyro_range_to_lsm6_range(uint16_t range)
{
    switch (range)
    {
    case 250:
        driver_to_mdps = reg::from_fs250_to_mdps;
        return reg::v_250dps;
    case 500:
        driver_to_mdps = reg::from_fs500_to_mdps;
        return reg::v_500dps;
    case 1000:
        driver_to_mdps = reg::from_fs1000_to_mdps;
        return reg::v_1000dps;
    default:
    case 2000:
        driver_to_mdps = reg::from_fs2000_to_mdps;
        return reg::v_2000dps;
    case 4000:
        driver_to_mdps = reg::from_fs4000_to_mdps;
        return reg::v_4000dps;
    }
}

static reg::data_rate_t sampling_rate_to_lsm6_rate(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return reg::v_ODR_AT_60Hz;
    case 120:
        return reg::v_ODR_AT_120Hz;
    case 240:
        return reg::v_ODR_AT_240Hz;
    case 480:
        return reg::v_ODR_AT_480Hz;
    case 960:
        return reg::v_ODR_AT_960Hz;
    case 1920:
        return reg::v_ODR_AT_1920Hz;
    case 3840:
        return reg::v_ODR_AT_3840Hz;
    case 7680:
        return reg::v_ODR_AT_7680Hz;
    default:
        return reg::v_ODR_AT_480Hz;
    }
}

static reg::sflp_data_rate_t sampling_rate_to_sflp_rate(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return reg::v_SFLP_60Hz;
    case 120:
        return reg::v_SFLP_120Hz;
    case 240:
        return reg::v_SFLP_240Hz;
    case 480:
    case 960:
    case 1920:
    case 3840:
    case 7680:
    default:
        return reg::v_SFLP_480Hz;
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

// FIFO batching rate must track the ODR, otherwise the FIFO is filled at a
// different rate than acc/gyro are produced. (XL and GY batch enums share the
// same numeric values, so one mapping serves both.)
static reg::fifo_xl_batch_t sampling_rate_to_batching(uint16_t rate)
{
    switch (rate)
    {
    case 60:
        return reg::v_XL_BATCHED_AT_60Hz;
    case 120:
        return reg::v_XL_BATCHED_AT_120Hz;
    case 240:
        return reg::v_XL_BATCHED_AT_240Hz;
    case 480:
        return reg::v_XL_BATCHED_AT_480Hz;
    case 960:
        return reg::v_XL_BATCHED_AT_960Hz;
    default:
        return reg::v_XL_BATCHED_AT_120Hz;
    }
}

static float_t npy_half_to_float(uint16_t h)
{
    union
    {
        float_t ret;
        uint32_t retbits;
    } conv;
    conv.retbits = reg::from_f16_to_f32(h);
    return conv.ret;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    (void)handle;
    if (!ready())
        return -1;
    // len == 0 is a command-only frame (the pre-seam code sent exactly the
    // register byte); both rtos backends emit it as such.
    const uint8_t *tx = len == 0 ? nullptr : bufp;
    if (s_spi == nullptr || !s_spi->transfer_cmd(reg & 0x7F, tx, nullptr, len))
    {
        status = imu::InitStatus::TransportError;
        RTOS_LOGE(TAG, "SPI write error, reg 0x%02x len %u", reg, (unsigned)len);
        return -1;
    }
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    (void)handle;
    if (!ready() && !probing)
        return -1;
    if (len == 0)
    {
        // Command-only frame, same as the pre-seam single register byte.
        if (s_spi == nullptr || !s_spi->transfer_cmd(reg | 0x80, nullptr, nullptr, 0))
        {
            status = imu::InitStatus::TransportError;
            RTOS_LOGE(TAG, "SPI read error, reg 0x%02x len 0", reg);
            return -1;
        }
        return 0;
    }

    // 0xFF dummies on MOSI during the data phase, matching the pre-seam
    // framing byte for byte (the sensor ignores MOSI while it drives MISO).
    uint8_t tx_buf[len];
    memset(tx_buf, 0xFF, len);

    if (s_spi == nullptr || !s_spi->transfer_cmd(reg | 0x80, tx_buf, bufp, len))
    {
        status = imu::InitStatus::TransportError;
        RTOS_LOGE(TAG, "SPI read error, reg 0x%02x len %u", reg, (unsigned)len);
        return -1;
    }
    return 0;
}

// Optional (may be required by driver)
static void platform_delay(uint32_t millisec)
{
    rtos::time::sleep_for(rtos::time::Millis(millisec));
}

static void setup_sampling_rate()
{

    reg::block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    reg::xl_full_scale_set(&dev_ctx, acc_range_to_lsm6_range(cfg.accelRange));
    reg::gy_full_scale_set(&dev_ctx, gyro_range_to_lsm6_range(cfg.gyroRange));

    reg::xl_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    reg::gy_data_rate_set(&dev_ctx, sampling_rate_to_lsm6_rate(cfg.sampleRate));
    reg::sflp_data_rate_set(&dev_ctx, sampling_rate_to_sflp_rate(cfg.sampleRate));

    reg::timestamp_set(&dev_ctx, PROPERTY_ENABLE);
    reg::sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
}

static void setup_fifo()
{
    reg::fifo_mode_set(&dev_ctx, reg::v_BYPASS_MODE);
    reg::fifo_watermark_set(&dev_ctx, cfg.fifoWatermark);
    const reg::fifo_xl_batch_t batch = sampling_rate_to_batching(cfg.sampleRate);
    reg::fifo_xl_batch_set(&dev_ctx, batch);
    reg::fifo_gy_batch_set(&dev_ctx, (reg::fifo_gy_batch_t)batch);
    reg::fifo_timestamp_batch_set(&dev_ctx, reg::v_TMSTMP_DEC_1);
    // Reapply the explicit opt-in after reset/configuration, including OFF
    // when a consumer disables it. FIFO bypass/stream retains this setting.
    const auto temperatureBatch =
        cfg.fifoTemperature ? reg::v_TEMP_BATCHED_AT_1Hz875 : reg::v_TEMP_NOT_BATCHED;
    if (reg::fifo_temp_batch_set(&dev_ctx, temperatureBatch) != 0)
        RTOS_LOGE(TAG, "FIFO temperature batching setup failed");
    reg::fifo_sflp_raw_t fifo_sflp = {};
    fifo_sflp.game_rotation = 1;
    fifo_sflp.gravity = 1;
#ifdef USE_GBIAS
    fifo_sflp.gbias = 1;
#endif
    reg::fifo_sflp_batch_set(&dev_ctx, fifo_sflp);
    reg::fifo_mode_set(&dev_ctx, reg::v_STREAM_MODE);
}

static void clear_fifo()
{
    reg::fifo_mode_set(&dev_ctx, reg::v_BYPASS_MODE);
    reg::fifo_mode_set(&dev_ctx, reg::v_STREAM_MODE);
}
static void setup_tapping()
{
    reg::interrupt_mode_t irq = {};
    reg::tap_detection_t tap = {};
    reg::tap_thresholds_t tap_ths = {};
    reg::tap_time_windows_t tap_win = {};
    irq.enable = 1;
    irq.lir = 0;
    reg::interrupt_enable_set(&dev_ctx, irq);
    tap.tap_z_en = 1;
    tap.tap_x_en = 1;
    tap.tap_y_en = 0;
    reg::tap_detection_set(&dev_ctx, tap);
    tap_ths.z = 0x1f;
    tap_ths.x = 0x1f;
    tap_ths.y = 0x1f;
    reg::tap_thresholds_set(&dev_ctx, tap_ths);
    tap_win.tap_gap = 3;
    tap_win.shock = 2;
    tap_win.quiet = 3;
    reg::tap_time_windows_set(&dev_ctx, tap_win);
    reg::tap_axis_priority_set(&dev_ctx, reg::v_XZY);

    reg::tap_mode_set(&dev_ctx, reg::v_BOTH_SINGLE_DOUBLE);
}

void impl_read_sources(imu::Sources *sources)
{
    if (!ready())
        return;
    reg::all_sources_t status{};
    if (reg::all_sources_get(&dev_ctx, &status) != 0)
        return;

    if (status.single_tap)
    {
        sources->tap_single = true;
        // printf("Single tap detected, sign %d, x %d, y %d, z %d\n", status.tap_sign, status.tap_x,
        // status.tap_y, status.tap_z);
    }
    if (status.double_tap)
    {
        sources->tap_double = true;
        // printf("Double tap detected, sign %d, x %d, y %d, z %d\n", status.tap_sign, status.tap_x,
        // status.tap_y, status.tap_z);
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
int impl_fifo_data_available()
{
    if (!ready())
        return -1;
    reg::fifo_status_t fifo_status{};
    if (reg::fifo_status_get(&dev_ctx, &fifo_status) != 0)
    {
        RTOS_LOGE(TAG, "FIFO status read error");
        return -1;
    }
    if (fifo_status.fifo_ovr == 1)
    {
        // Samples were lost and the FIFO may hold torn packets (a word from
        // the middle of a sample group as the oldest entry). Resync to a
        // clean stream instead of draining garbage; the cost is the buffered
        // backlog, bounded by the FIFO depth (~0.5 s at 120 Hz).
        overrun_count++;
        RTOS_LOGE(TAG, "FIFO overrun #%u, clearing FIFO to resync", (unsigned)overrun_count);
        clear_fifo();
        return 0;
    }
    return fifo_status.fifo_level;
}

uint32_t impl_fifo_overrun_count() { return overrun_count; }

int impl_fifo_read_element(reg::fifo_out_raw_t &f_data)
{
    if (!ready())
        return -1;
    return reg::fifo_out_raw_get(&dev_ctx, &f_data);
}

void impl_fifo_process_xl(reg::fifo_out_raw_t &f_data, Vector3<float> &acc)
{
    int16_t data[3];
    memcpy(data, f_data.data, sizeof(data));

    acc.x = driver_to_mg(data[0]) * 0.001;
    acc.y = driver_to_mg(data[1]) * 0.001;
    acc.z = driver_to_mg(data[2]) * 0.001;
}

void impl_fifo_process_gyro(reg::fifo_out_raw_t &f_data, Vector3<float> &gyro)
{
    int16_t data[3];
    memcpy(data, f_data.data, sizeof(data));

    gyro.x = driver_to_mdps(data[0]) * 0.001;
    gyro.y = driver_to_mdps(data[1]) * 0.001;
    gyro.z = driver_to_mdps(data[2]) * 0.001;
}

void impl_fifo_process_timestamp(reg::fifo_out_raw_t &f_data, float &timestamp)
{
    int32_t ts;
    memcpy(&ts, f_data.data, sizeof(ts));
    timestamp = timestamp_lsb_sec * ts; // Convert to seconds using calibrated LSB
}
void impl_fifo_process_gravity(reg::fifo_out_raw_t &f_data, Vector3<float> &gravity)
{
    int16_t axis[3];
    memcpy(axis, f_data.data, sizeof(axis));
    gravity.x = reg::from_sflp_to_mg(axis[0]) * 0.001;
    gravity.y = reg::from_sflp_to_mg(axis[1]) * 0.001;
    gravity.z = reg::from_sflp_to_mg(axis[2]) * 0.001;
}

static inline uint16_t u16_le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

void impl_fifo_process_temperature(reg::fifo_out_raw_t &f_data, float &celsius)
{
    // ST OUT_TEMP_L/H: signed little-endian, 256 LSB/C with a 25 C offset.
    const int16_t raw = static_cast<int16_t>(u16_le(f_data.data));
    celsius = reg::from_lsb_to_celsius(raw);
}

#if IMU_LSM6DSV_PART == 80
// f_data1 and f_data2 must be the two consecutive 0x13 words for one sample
void impl_fifo_process_sflp_game_rotation(reg::fifo_out_raw_t &f_data1,
                                          reg::fifo_out_raw_t &f_data2, Quaternion<float> &q)
{
    auto sensor_id = [](uint8_t tag) { return tag & 0x1F; }; // 5-bit sensor code

    // Basic validation: both must be game-rotation vector (0x13)
    if (sensor_id(f_data1.tag) != 0x13 || sensor_id(f_data2.tag) != 0x13)
    {
        // Not our packets; zero out and return
        q = {0, 0, 0, 1};
        return;
    }

    // Markers in X (little-endian): 0x0000 = first word, 0x0001 = second word
    uint16_t m1 = u16_le(&f_data1.data[0]);
    uint16_t m2 = u16_le(&f_data2.data[0]);

    // Optional sanity: they usually share the same TAG_CNT
    // (don’t hard-fail if they don’t; just a useful check)
    // Counter equality was never enforced by the 80X wrapper.

    uint16_t hx = 0, hy = 0, hz = 0, hw = 0;

    if (m1 == 0x0000 && m2 == 0x0001)
    {
        // first then second (expected order)
        hx = u16_le(&f_data1.data[2]); // QUATX
        hy = u16_le(&f_data1.data[4]); // QUATY
        hz = u16_le(&f_data2.data[2]); // QUATZ
        hw = u16_le(&f_data2.data[4]); // QUATW
    }
    else if (m1 == 0x0001 && m2 == 0x0000)
    {
        // reversed arguments — still handle
        hx = u16_le(&f_data2.data[2]); // QUATX
        hy = u16_le(&f_data2.data[4]); // QUATY
        hz = u16_le(&f_data1.data[2]); // QUATZ
        hw = u16_le(&f_data1.data[4]); // QUATW
    }
    else
    {
        // Unexpected markers; bail safely
        q = {0, 0, 0, 1};
        return;
    }

    // Convert half -> float (use your npy_half_to_float)
    q.x = npy_half_to_float(hx);
    q.y = npy_half_to_float(hy);
    q.z = npy_half_to_float(hz);
    q.w = npy_half_to_float(hw);

    // Guard + normalize
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
        q = {0, 0, 0, 1};
        return;
    }
    float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (n2 > 0.0f)
    {
        float inv = 1.0f / std::sqrt(n2);
        q.x *= inv;
        q.y *= inv;
        q.z *= inv;
        q.w *= inv;
    }
}

#endif

float impl_get_timestamp_resolution()
{
    if (!ready())
        return 0;
    int8_t freq_fine = 0;
    if (reg::odr_cal_reg_get(&dev_ctx, &freq_fine) != 0)
    {
        printf("!!! Error getting odr calibration register !!!\n");
        return 0;
    }
    // Timestamp clock is 46.08 kHz trimmed by FREQ_FINE (datasheet AN): Tlsb = 1 / (46.08 kHz * (1
    // + 0.0013 * freq_fine))
    float clk_hz = 46080.0f * (1.0f + 0.0013f * (float)freq_fine);
    return 1.0f / clk_hz;
}

void impl_init_spi(rtos::SpiDevice *device)
{
    s_spi = device;
    status = imu::InitStatus::NotInitialized;
}

void impl_start_sampling(bool start)
{
    if (!ready())
        return;
    printf("##### %s %s sampling #####\n", imu::part_name, start ? "start" : "stop");

    if (start)
    {
        // Re-arm the sensors: the stop path below powers them down, so
        // restore ODRs, SFLP and batching before draining begins. Re-running
        // the setup helpers is idempotent when nothing was stopped.
        setup_sampling_rate();
        clear_fifo();

        int8_t freq_fine = 0;
        reg::odr_cal_reg_get(&dev_ctx, &freq_fine);
        float tactual = impl_get_timestamp_resolution();
        float odractual =
            7680.0 * (1 + 0.0013 * (float)freq_fine) / sampling_rate_to_odrcoeff(cfg.sampleRate);
        RTOS_LOGI(TAG,
                  "%s started, sample rate: %d, acc range: %d, gyro range: %d timestamp res %fs, "
                  "odr actual %fHz",
                  imu::part_name, cfg.sampleRate, cfg.accelRange, cfg.gyroRange, tactual,
                  odractual);
    }
    else
    {
        // Actually power the sensors down between sessions. This branch used
        // to be a bare printf, so the gyro + SFLP ran at 120 Hz from boot
        // forever — idle battery life was gyro-dominated (~1 mA) regardless
        // of light sleep. Tap detection is currently disabled, so the XL
        // isn't needed at idle either; if tap returns, keep the XL running
        // at a low ODR instead of OFF.
        reg::fifo_mode_set(&dev_ctx, reg::v_BYPASS_MODE);
        reg::sflp_game_rotation_set(&dev_ctx, PROPERTY_DISABLE);
        reg::xl_data_rate_set(&dev_ctx, reg::v_ODR_OFF);
        reg::gy_data_rate_set(&dev_ctx, reg::v_ODR_OFF);
    }
}

void impl_start_tapping(bool start)
{
    if (!ready())
        return;
    // printf("##### LSM6DSV80X start tapping, %d #####\n", start);
    // setup_sampling_rate();
    if (start)
        setup_tapping();
    uint8_t property_enable = start ? PROPERTY_ENABLE : PROPERTY_DISABLE;
    reg::pin_int_route_t pin_int2 = {};
    pin_int2.double_tap = property_enable;
    pin_int2.single_tap = property_enable;
    // pin_int2.sleep_change = property_enable;
    reg::pin_int2_route_set(&dev_ctx, &pin_int2);
}

static void impl_reset()
{
#if IMU_LSM6DSV_PART == 32
    if (reg::sw_por(&dev_ctx) != 0)
        status = imu::InitStatus::TransportError;
#else

    reg::reset_t rst = reg::v_READY;

    if (-1 ==
        reg::reset_set(&dev_ctx, (reg::reset_t)(reg::v_RESTORE_CTRL_REGS | reg::v_GLOBAL_RST)))
        RTOS_LOGE(TAG, "LSM6DSV80X reset failed");

    const int sleep_time = 5000;
    int total_sleep_time = 0;
    do
    {
        rtos::time::sleep_for(rtos::time::Millis(sleep_time / 1000));
        if (reg::reset_get(&dev_ctx, &rst) != 0)
            return;
        total_sleep_time += sleep_time;
    } while (rst != reg::v_READY && total_sleep_time < 1000000);
    if (rst != reg::v_READY)
        RTOS_LOGE(TAG, "LSM6DSV80X reset failed");
#endif
}

void impl_config(DriverConfig *newCfg)
{
    if (!ready())
        return;
    cfg = *newCfg;
    if (debug)
    {
        // printf("##### LSM6DSV80X config #####\n");
        printf("%s config:\n", imu::part_name);
        printf("  sample rate: %d\n", cfg.sampleRate);
        printf("  accel range: %d\n", cfg.accelRange);
        printf("  gyro range: %d\n", cfg.gyroRange);
        printf("  batching: %d\n", cfg.batching);
        printf("  fifo watermark: %d\n", cfg.fifoWatermark);
        printf("  fifo mode: %d\n", cfg.fifoMode);
        printf("  timeout: %u\n", static_cast<unsigned>(cfg.timeout));
    }
    impl_reset();
    if (!ready())
        return;
    setup_sampling_rate();
    timestamp_lsb_sec = impl_get_timestamp_resolution();
    printf("%s timestamp LSB: %fus\n", imu::part_name, timestamp_lsb_sec * 1e6);
    setup_fifo();
    // setup_filter();
}

void impl_set_debug(bool d) { debug = d; }

imu::InitStatus impl_init()
{
    status = imu::InitStatus::NotInitialized;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = s_spi;
    uint8_t observed = 0;
    probing = true;
    const auto result = reg::device_id_get(&dev_ctx, &observed);
    probing = false;
    if (result != 0)
    {
        status = imu::InitStatus::TransportError;
        RTOS_LOGE(TAG, "Expected %s, observed unreadable part", imu::part_name);
    }
    else if (observed != imu::expected_id)
    {
        status = imu::InitStatus::WrongPart;
        RTOS_LOGE(TAG, "Expected %s, observed %s", imu::part_name,
                  observed == 0x70   ? "LSM6DSV32X"
                  : observed == 0x73 ? "LSM6DSV80X"
                                     : "unknown part");
    }
    else
        status = imu::InitStatus::Ready;
    return status;
}
} // namespace
