#include "lsm6dsv80x.hpp"
#include "mock_spi.hpp"
#include <iomanip>
#ifdef NEUTRAL_TRACE
#include "neutral_trace_adapter.hpp"
#endif
int main()
{
    rtos::SpiDevice spi;
    mock::reset(0x73);
    lsm6dsv80x_init_spi(&spi);
    lsm6dsv80x_init();
    lsm6dsv80x_cfg_t c{120, 4,   2000, LSM6DSV80X_XL_BATCHED_AT_120Hz, 32, LSM6DSV80X_STREAM_MODE,
                       0,   true};
    lsm6dsv80x_config(&c);
    lsm6dsv80x_start_sampling(true);
    lsm6dsv80x_fifo_out_raw_t w{};
    Vector3<float> v;
    float f = 0;
    auto scalar = [&](const char *tag, float n)
    { std::cout << "VALUE " << tag << ' ' << std::hexfloat << n << '\n'; };
    auto vec = [&](const char *tag)
    {
        scalar(tag, v.x);
        scalar(tag, v.y);
        scalar(tag, v.z);
    };
    const std::array<std::array<uint8_t, 7>, 6> words{
        {{uint8_t(2 << 3), 0x34, 0x12, 0xff, 0xff, 0, 0x80},
         {uint8_t(1 << 3), 0, 0x80, 0xff, 0x7f, 0, 0},
         {uint8_t(0x17 << 3), 0, 0, 0, 0, 0, 0x40},
         {uint8_t(3 << 3), 0x80, 0xfe, 0xa5, 0xa5, 0xa5, 0xa5},
         {uint8_t(4 << 3), 0x78, 0x56, 0x34, 0x12, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}}};
    for (unsigned i = 0; i < words.size(); ++i)
    {
        mock::fifo.push_back(words[i]);
        mock::require(lsm6dsv80x_fifo_read_element(w) == 0, "fifo transport");
        switch (i)
        {
        case 0:
            lsm6dsv80x_fifo_process_xl(w, v);
            vec("xl");
            break;
        case 1:
            lsm6dsv80x_fifo_process_gyro(w, v);
            vec("gy");
            break;
        case 2:
            lsm6dsv80x_fifo_process_gravity(w, v);
            vec("gravity");
            break;
        case 3:
            lsm6dsv80x_fifo_process_temperature(w, f);
            scalar("temp", f);
            break;
        case 4:
            lsm6dsv80x_fifo_process_timestamp(w, f);
            scalar("timestamp", f);
            break;
        default:
            break;
        }
    }
    lsm6dsv80x_fifo_out_raw_t a{}, b{};
    a.tag = b.tag = decltype(a)::LSM6DSV80X_SFLP_GAME_ROTATION_VECTOR_TAG;
    a.data[3] = 0x38;
    b.data[0] = 1;
    b.data[4] = 0xee;
    b.data[5] = 0x3a;
    Quaternion<float> q;
    lsm6dsv80x_fifo_process_sflp_game_rotation(a, b, q);
    scalar("qx", q.x);
    scalar("qy", q.y);
    scalar("qz", q.z);
    scalar("qw", q.w);
    lsm6dsv80x_fifo_process_sflp_game_rotation(b, a, q);
    scalar("reverse x", q.x);
    scalar("reverse w", q.w);
    mock::banks[0][0x1b] = 7;
    scalar("available", lsm6dsv80x_fifo_data_available());
    mock::banks[0][0x1c] = 0x40;
    scalar("overrun available", lsm6dsv80x_fifo_data_available());
    scalar("overruns", lsm6dsv80x_fifo_overrun_count());
    mock::require(lsm6dsv80x_fifo_overrun_count() == 1, "overrun path exercised");
    mock::banks[0][0x1c] = 0;
    lsm6dsv80x_start_sampling(false);
    lsm6dsv80x_start_sampling(true);
    lsm6dsv80x_start_sampling(false);
    for (const auto &event : mock::trace)
    {
        std::cout << "TRACE";
        for (auto n : event)
            std::cout << ' ' << n;
        std::cout << '\n';
    }
}
