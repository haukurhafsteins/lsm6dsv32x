#pragma once
#include "Quaternion.hpp"
#include "rtos/Spi.hpp"
#include "vectors.h"
#include <array>
#include <cstddef>
#include <cstdint>

// Must be identical in the driver and every consumer. Legacy builds select 80.
#ifndef IMU_LSM6DSV_PART
#define IMU_LSM6DSV_PART 80
#endif
#if IMU_LSM6DSV_PART != 80 && IMU_LSM6DSV_PART != 32
#error "IMU_LSM6DSV_PART must be 80 or 32"
#endif
namespace imu
{
inline constexpr std::size_t rotation_word_count = IMU_LSM6DSV_PART == 32 ? 1 : 2;
inline constexpr uint8_t expected_id = IMU_LSM6DSV_PART == 32 ? 0x70 : 0x73;
inline constexpr const char *part_name = IMU_LSM6DSV_PART == 32 ? "LSM6DSV32X" : "LSM6DSV80X";
enum class InitStatus
{
    NotInitialized,
    Ready,
    WrongPart,
    TransportError
};
enum class Rate : uint16_t
{
    Hz60 = 60,
    Hz120 = 120,
    Hz240 = 240,
    Hz480 = 480,
    Hz960 = 960
};
enum class AccelRange : uint8_t
{
    G2 = 2,
    G4 = 4,
    G8 = 8,
    G16 = 16,
    G32 = 32
};
enum class GyroRange : uint16_t
{
    Dps250 = 250,
    Dps500 = 500,
    Dps1000 = 1000,
    Dps2000 = 2000,
    Dps4000 = 4000
};
// The existing product contract is stream FIFO with batching derived from ODR.
enum class FifoMode
{
    Stream
};
enum class Batching
{
    FollowSampleRate
};
struct Config
{
    Rate sampleRate = Rate::Hz120;
    AccelRange accelRange = AccelRange::G4;
    GyroRange gyroRange = GyroRange::Dps2000;
    Batching batching = Batching::FollowSampleRate;
    uint16_t fifoWatermark = 32;
    FifoMode fifoMode = FifoMode::Stream;
    bool fifoTemperature = false;
};
enum class Tag : uint8_t
{
    Empty = 0,
    Gyroscope = 1,
    Accelerometer = 2,
    Temperature = 3,
    Timestamp = 4,
    ConfigurationChange = 5,
    GameRotation = 0x13,
    GyroscopeBias = 0x16,
    Gravity = 0x17,
    Unknown = 0xff
};
struct FifoWord
{
    Tag tag = Tag::Empty;
    uint8_t counter = 0;
    std::array<uint8_t, 6> data{};
};
struct Sources
{
    bool tap_x = false, tap_y = false, tap_z = false, tap_single = false, tap_double = false,
         tap_sign = false, sleep_change = false;
};
enum class DecodeResult
{
    Ready,
    NeedMore,
    Invalid
};
// Consumer skips temperature independently, supplies exactly rotation_word_count
// motion words, and retains its drop accounting. Extra words are never consumed.
// Reset at discontinuity, overrun, configuration and session boundaries.
class RotationDecoder
{
    Quaternion<float> previous_{};
    bool has_previous_ = false;

  public:
    void reset() noexcept { has_previous_ = false; }
    DecodeResult decode(const FifoWord *words, std::size_t count, Quaternion<float> &q);
};
void init_spi(rtos::SpiDevice *device);
InitStatus init();
InitStatus readiness();
bool config(const Config &cfg);
void start_sampling(bool start);
void start_tapping(bool start);
void sleep();
void set_debug(bool enabled);
void read_sources(Sources &sources);
float get_timestamp_resolution();
int fifo_data_available();
uint32_t fifo_overrun_count();
int fifo_read_element(FifoWord &word);
// Pure codecs; call after successful config for selected full-scale conversion.
void process_xl(const FifoWord &word, Vector3<float> &value);
void process_gyro(const FifoWord &word, Vector3<float> &value);
void process_gravity(const FifoWord &word, Vector3<float> &value);
void process_temperature(const FifoWord &word, float &celsius);
void process_timestamp(const FifoWord &word, float &seconds);
} // namespace imu
