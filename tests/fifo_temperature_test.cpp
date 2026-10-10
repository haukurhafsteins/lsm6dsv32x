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
    return 0;
}

void checkFifo()
{
    lsm6dsv80x_fifo_temp_batch_t temperature;
    lsm6dsv80x_fifo_xl_batch_t accel;
    lsm6dsv80x_fifo_gy_batch_t gyro;
    lsm6dsv80x_fifo_timestamp_batch_t timestamp;
    lsm6dsv80x_fifo_mode_t mode;
    uint8_t watermark;
    require(lsm6dsv80x_fifo_temp_batch_get(&dev_ctx, &temperature) == 0,
            "temperature batch register read failed");
    require(temperature == LSM6DSV80X_TEMP_BATCHED_AT_1Hz875,
            "FIFO temperature must batch at 1.875 Hz after setup/restart");
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
    // setup_fifo is also called after the wrapper resets/reconfigures the chip.
    for (const uint16_t rate : {60, 120, 240}) {
        registers.fill(0);
        cfg.sampleRate = rate;
        setup_fifo();
        checkFifo();
        lsm6dsv80x_start_sampling(false);
        lsm6dsv80x_start_sampling(true);
        checkFifo();
        clear_fifo(); // overrun recovery must retain temperature configuration
        checkFifo();
    }
    require(lsm6dsv80x_from_lsb_to_celsius(0) == 25.0f, "zero offset changed");
    require(lsm6dsv80x_from_lsb_to_celsius(256) == 26.0f, "positive scale changed");
    require(lsm6dsv80x_from_lsb_to_celsius(-256) == 24.0f, "signed scale changed");
    registers.fill(0);
    failTemperatureWrite = true;
    setup_fifo();
    lsm6dsv80x_fifo_temp_batch_t temperature;
    require(lsm6dsv80x_fifo_temp_batch_get(&dev_ctx, &temperature) == 0 &&
            temperature == LSM6DSV80X_TEMP_NOT_BATCHED, "failed enable fabricated readiness");
    require(errorLogs == 1, "temperature configuration failure must be observable");
    std::cout << "FIFO temperature setup, restart, recovery and conversion passed\n";
}
