#include "imu_lsm6dsv.hpp"
#include "mock_spi.hpp"
#include <cmath>
using mock::require;
using namespace imu;
int main()
{
    rtos::SpiDevice spi;
    mock::reset(0);
    init_spi(&spi);
    require(init() == InitStatus::WrongPart, "wrong identity must fail init");
    Sources sources;
    const auto stopped = mock::trace.size();
    Config cfg;
    require(!config(cfg), "failed init rejects config");
    start_sampling(true);
    start_sampling(false);
    start_sampling(true);
    start_tapping(true);
    read_sources(sources);
    sleep();
    require(get_timestamp_resolution() == 0, "failed init has no clock");
    FifoWord word;
    require(fifo_read_element(word) < 0, "failed init must not read samples");
    require(fifo_data_available() < 0, "failed init must not poll FIFO");
    require(mock::trace.size() == stopped, "failed init must latch: no subsequent I/O");
    mock::reset(expected_id);
    mock::fail = true;
    init_spi(&spi);
    require(init() == InitStatus::TransportError, "transport error must fail init");
    mock::fail = false;
    const auto failed = mock::trace.size();
    config(cfg);
    start_sampling(true);
    require(mock::trace.size() == failed, "transport failure must latch");
    mock::reset(expected_id);
    init_spi(&spi);
    require(init() == InitStatus::Ready, "selected chip must initialize");
    cfg.fifoTemperature = true;
    require(config(cfg), "product configuration accepted");
    start_sampling(true);
    require((mock::banks[0][0x10] & 15) == 6 && (mock::banks[0][0x11] & 15) == 6,
            "120 Hz both sensors");
    require((mock::banks[0][0x0a] & 0x30) == 0x10, "temperature opt-in");
    start_sampling(false);
    require((mock::banks[0][0x10] & 15) == 0 && (mock::banks[0][0x11] & 15) == 0,
            "stop powers down");
    start_sampling(true);
    require((mock::banks[0][0x10] & 15) == 6, "restart re-arms");
    mock::banks[0][0x1c] = 0x40;
    const auto before = fifo_overrun_count();
    require(fifo_data_available() == 0 && fifo_overrun_count() == before + 1,
            "overrun cleared/count incremented");
    mock::banks[0][0x1c] = 0;
    mock::fifo.push_back({uint8_t((2 << 3) | (3 << 1)), 0x00, 0x01, 0x00, 0xff, 0, 0});
    require(fifo_read_element(word) == 0 && word.tag == Tag::Accelerometer && word.counter == 3,
            "tag extraction");
    Vector3<float> acc;
    process_xl(word, acc);
    require(std::abs(acc.x - 0.031232f) < 1e-7f && acc.y == -acc.x, "4g scale");
    // Calibration and pure codecs use the same physical units on both parts.
    mock::banks[0][0x4f] = 10;
    require(std::abs(get_timestamp_resolution() - 1.0f / (46080.0f * 1.013f)) < 1e-10f,
            "signed frequency calibration");
    mock::banks[0][0x4f] = uint8_t(-10);
    require(std::abs(get_timestamp_resolution() - 1.0f / (46080.0f * 0.987f)) < 1e-10f,
            "negative frequency calibration");
    word.data = {0x00, 0x01, 0x00, 0xff, 0, 0};
    Vector3<float> gyro;
    process_gyro(word, gyro);
    require(std::abs(gyro.x - 17.92f) < 1e-5f && gyro.y == -gyro.x, "2000dps conversion");
    float temperature = 0;
    word.data = {0x80, 0xfe, 0xa5, 0xa5, 0xa5, 0xa5};
    process_temperature(word, temperature);
    require(temperature == 23.5f, "signed temperature ignores unrelated bytes");
    const auto previousTrace = mock::trace.size();
    cfg.accelRange = IMU_LSM6DSV_PART == 32 ? AccelRange::G2 : AccelRange::G32;
    require(!config(cfg) && mock::trace.size() == previousTrace,
            "other part full scale rejected before I/O");
    cfg.accelRange = AccelRange::G4;
    cfg.fifoTemperature = false;
    require(config(cfg), "temperature can be disabled");
    require((mock::banks[0][0x0a] & 0x30) == 0, "temperature disabled after config");
    mock::fail = true;
    require(fifo_read_element(word) < 0, "transport failure during read");
    mock::fail = false;
    const auto failedRead = mock::trace.size();
    start_sampling(true);
    require(fifo_read_element(word) < 0 && mock::trace.size() == failedRead,
            "transport failure cannot restart implicitly");
    // Explicit reinitialization is the only recovery from a latched failure.
    require(init() == InitStatus::Ready && config(cfg), "explicit recovery checks ID again");
    std::cout << "driver contract passed\n";
}
