#pragma once
#include "rtos/Log.hpp"
#include "rtos/Spi.hpp"
#include "rtos/time.hpp"
#include <array>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <vector>
namespace mock
{
inline std::array<std::array<uint8_t, 256>, 3> banks{};
inline unsigned bank = 0;
inline bool fail = false;
inline std::vector<std::vector<unsigned>> trace;
inline std::deque<std::array<uint8_t, 7>> fifo;
inline void require(bool v, const char *why)
{
    if (!v)
    {
        std::cerr << why << '\n';
        std::exit(1);
    }
}
inline void reset(uint8_t id)
{
    banks = {};
    bank = 0;
    fail = false;
    trace.clear();
    fifo.clear();
    banks[0][0x0f] = id;
}
} // namespace mock
namespace rtos
{
void SpiDevice::deinit() {}
bool SpiDevice::transfer_cmd(uint16_t cmd, const uint8_t *tx, uint8_t *rx, size_t len)
{
    using namespace mock;
    std::vector<unsigned> event{cmd, static_cast<unsigned>(len)};
    for (size_t i = 0; i < len; ++i)
        event.push_back(tx ? tx[i] : 256);
    trace.push_back(event);
    if (fail)
        return false;
    const auto addr = cmd & 0x7f;
    if (cmd & 0x80)
    {
        if (addr == 0x78 && len == 7 && !fifo.empty())
        {
            std::memcpy(rx, fifo.front().data(), 7);
            fifo.pop_front();
            return true;
        }
        if (len)
            std::memcpy(rx, banks[bank].data() + addr, len);
    }
    else if (len)
    {
        std::memcpy(banks[bank].data() + addr, tx, len);
        if (addr == 1)
        {
            if (tx[0] & 4)
            {
                auto id = banks[0][0x0f];
                banks = {};
                banks[0][0x0f] = id;
            }
            bank = (tx[0] & 0x80) ? 1 : ((tx[0] & 0x40) ? 2 : 0);
            for (auto &b : banks)
                b[1] = tx[0] & ~4;
        }
        if (addr == 0x12 && (tx[0] & 1))
            banks[0][addr] &= ~1;
    }
    return true;
}
void Log::log(LogLevel, const char *, const char *, ...) {}
} // namespace rtos
namespace rtos::time
{
void sleep_for(Millis) noexcept {}
} // namespace rtos::time
