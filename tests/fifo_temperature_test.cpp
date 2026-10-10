#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

// Exercise the actual wrapper setup and ST register codec without hardware.
#include "../lsm6dsv80x.cpp"

namespace {
unsigned errorLogs = 0;
bool failTemperatureWrite = false;
}

namespace rtos {
void Log::log(LogLevel level, const char *, const char *, ...)
{
    if (level == LogLevel::Error) ++errorLogs;
}
}
namespace rtos::time {
void sleep_for(Millis) noexcept {}
}

namespace {
std::array<uint8_t, 256> registers{};

void require(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

int32_t readRegister(void *, uint8_t address, uint8_t *bytes, uint16_t length)
{
    std::memcpy(bytes, registers.data() + address, length);
    return 0;
}

int32_t writeRegister(void *, uint8_t address, const uint8_t *bytes, uint16_t length)
{
    // Temperature shares FIFO_CTRL4 with timestamp decimation and FIFO mode.
    // Reject only an attempt to enable temperature, preserving unrelated writes.
    if (failTemperatureWrite && address == LSM6DSV80X_FIFO_CTRL4 &&
        length == 1 && (bytes[0] & 0x30) != 0)
        return -1;
    std::memcpy(registers.data() + address, bytes, length);
    // The hardware clears reset bits after restoring control registers.
    if (address == LSM6DSV80X_FUNC_CFG_ACCESS && length == 1 &&
        (bytes[0] & 0x04) != 0)
        registers.fill(0);
    return 0;
}

void checkFifo(bool temperatureEnabled = true)
{
    lsm6dsv80x_fifo_temp_batch_t temperature;
    lsm6dsv80x_fifo_xl_batch_t accel;
    lsm6dsv80x_fifo_gy_batch_t gyro;
    lsm6dsv80x_fifo_timestamp_batch_t timestamp;
    lsm6dsv80x_fifo_mode_t mode;
    uint8_t watermark;
    require(lsm6dsv80x_fifo_temp_batch_get(&dev_ctx, &temperature) == 0,
            "temperature batch register read failed");
    require(temperature == (temperatureEnabled ? LSM6DSV80X_TEMP_BATCHED_AT_1Hz875
                                              : LSM6DSV80X_TEMP_NOT_BATCHED),
            "FIFO temperature batching does not match opt-in configuration");
    require(lsm6dsv80x_fifo_xl_batch_get(&dev_ctx, &accel) == 0 &&
            accel == sampling_rate_to_batching(cfg.sampleRate), "accel batching changed");
    require(lsm6dsv80x_fifo_gy_batch_get(&dev_ctx, &gyro) == 0 &&
            static_cast<unsigned>(gyro) == static_cast<unsigned>(accel), "gyro batching changed");
    require(lsm6dsv80x_fifo_timestamp_batch_get(&dev_ctx, &timestamp) == 0 &&
            timestamp == LSM6DSV80X_TMSTMP_DEC_1, "timestamp batching changed");
    require(lsm6dsv80x_fifo_mode_get(&dev_ctx, &mode) == 0 &&
            mode == LSM6DSV80X_STREAM_MODE, "FIFO stream mode changed");
    require(lsm6dsv80x_fifo_watermark_get(&dev_ctx, &watermark) == 0 &&
            watermark == cfg.fifoWatermark, "FIFO watermark changed");
}
}

int main()
{
    dev_ctx.read_reg = readRegister;
    dev_ctx.write_reg = writeRegister;
    setup_fifo();
    checkFifo(false); // Existing consumers must retain the motion-only FIFO.
    lsm6dsv80x_cfg_t defaultConstructed;
    lsm6dsv80x_cfg_t zeroInitialized = {};
    lsm6dsv80x_cfg_t partialAggregate = {.sampleRate = 120};
    require(!defaultConstructed.fifoTemperature && !zeroInitialized.fifoTemperature &&
            !partialAggregate.fifoTemperature, "omitted temperature field must default off");
    // setup_fifo is also called after the wrapper resets/reconfigures the chip.
    for (const uint16_t rate : {60, 120, 240}) {
        registers.fill(0);
        cfg.sampleRate = rate;
        for (const bool enabled : {true, false, true}) {
            auto next = cfg;
            next.fifoTemperature = enabled;
            // Exercise explicit disabling even before a hardware reset.
            cfg = next;
            setup_fifo();
            checkFifo(enabled);
            lsm6dsv80x_config(&next);
            checkFifo(enabled);
            lsm6dsv80x_start_sampling(false);
            lsm6dsv80x_start_sampling(true);
            checkFifo(enabled);
            clear_fifo(); // overrun recovery retains the opt-in setting
            checkFifo(enabled);
        }
    }
    struct ConversionCase { uint8_t low; uint8_t high; float expected; };
    for (const auto test : std::array<ConversionCase, 7>{{
        {0x00, 0x00, 25.0f}, {0x00, 0x01, 26.0f}, {0x00, 0xff, 24.0f},
        {0x80, 0x01, 26.5f}, {0x80, 0xfe, 23.5f},
        {0x00, 0x80, -103.0f}, {0xff, 0x7f, 152.99609375f},
    }}) {
        lsm6dsv80x_fifo_out_raw_t word{};
        word.tag = lsm6dsv80x_fifo_out_raw_t::LSM6DSV80X_TEMPERATURE_TAG;
        std::memset(word.data, 0xa5, sizeof(word.data));
        word.data[0] = test.low;
        word.data[1] = test.high;
        float celsius = 0.0f;
        lsm6dsv80x_fifo_process_temperature(word, celsius);
        require(celsius == test.expected, "temperature bytes must decode signed little-endian");
    }
    registers.fill(0);
    failTemperatureWrite = true;
    setup_fifo();
    lsm6dsv80x_fifo_temp_batch_t temperature;
    require(lsm6dsv80x_fifo_temp_batch_get(&dev_ctx, &temperature) == 0 &&
            temperature == LSM6DSV80X_TEMP_NOT_BATCHED, "failed enable fabricated readiness");
    require(errorLogs == 1, "temperature configuration failure must be observable");
    std::cout << "FIFO temperature setup, restart, recovery and conversion passed\n";
}
