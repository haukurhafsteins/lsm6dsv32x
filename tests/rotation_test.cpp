#include "imu_lsm6dsv.hpp"
#include "mock_spi.hpp"
#include <cmath>
using namespace imu;
using mock::require;
void half(FifoWord &w, unsigned i, uint16_t h)
{
    w.data[2 * i] = h & 255;
    w.data[2 * i + 1] = h >> 8;
}
int main()
{
    RotationDecoder decoder;
    Quaternion<float> q;
    FifoWord words[3]{};
    words[0].tag = words[1].tag = Tag::GameRotation;
    if constexpr (rotation_word_count == 2)
    {
        half(words[1], 0, 1);
        half(words[1], 2, 0x3c00);
    }
    require(decoder.decode(words, rotation_word_count, q) == DecodeResult::Ready,
            "identity accepted");
    require(q.w == 1 && q.x == 0 && q.y == 0 && q.z == 0, "identity convention");
    require(decoder.decode(words, rotation_word_count - 1, q) == DecodeResult::NeedMore,
            "truncation detected");
    words[0].tag = Tag::Temperature;
    require(decoder.decode(words, rotation_word_count, q) == DecodeResult::Invalid,
            "interleaved temperature must be skipped by consumer, not rotation");
    words[0].tag = Tag::GameRotation;
    if constexpr (rotation_word_count == 1)
    {
        half(words[0], 0, 0x3800);
        require(decoder.decode(words, 1, q) == DecodeResult::Ready, "representative rotation");
        require(std::abs(q.x - 0.5f) < 1e-6f && std::abs(q.w - std::sqrt(0.75f)) < 1e-6f,
                "reconstruct w");
        half(words[0], 0, 0x3bff);
        require(decoder.decode(words, 1, q) == DecodeResult::Ready, "near pi");
        auto prev = q;
        half(words[0], 0, 0xbbff);
        require(decoder.decode(words, 1, q) == DecodeResult::Ready, "cross pi");
        require(q.dot(prev) > 0.99f, "hemisphere continuity");
        decoder.reset();
        require(decoder.decode(words, 1, q) == DecodeResult::Ready && q.w > 0 && q.x < 0,
                "reset discards continuity");
        half(words[0], 0, 0x39a9);
        half(words[0], 1, 0x39a9);
        require(decoder.decode(words, 1, q) == DecodeResult::Ready, "rounded norm over one");
        require(std::abs(q.magnitude() - 1) < 1e-6f && q.w == 0, "rounding normalized");
        half(words[0], 0, 0x4000);
        require(decoder.decode(words, 1, q) == DecodeResult::Invalid,
                "gross out of unit ball rejected");
        words[0].data = {};
        half(words[0], 0, 0x7c00);
        require(decoder.decode(words, 1, q) == DecodeResult::Invalid, "infinity rejected");
        half(words[0], 0, 0x7e00);
        require(decoder.decode(words, 1, q) == DecodeResult::Invalid, "NaN rejected");
        words[0].data = {};
        words[1].tag = Tag::Gyroscope;
        require(decoder.decode(words, 2, q) == DecodeResult::Ready,
                "32X consumes one word even with next sample present");
    }
    else
    {
        half(words[0], 1, 0x3800);
        half(words[1], 2, 0x3aee);
        require(decoder.decode(words, 2, q) == DecodeResult::Ready, "80X rotation");
        words[0].counter = 0;
        words[1].counter = 3;
        require(decoder.decode(words, 2, q) == DecodeResult::Ready,
                "80X counter mismatch remains tolerated");
        auto forward = q;
        std::swap(words[0], words[1]);
        require(decoder.decode(words, 2, q) == DecodeResult::Ready && q.dot(forward) > 0.999f,
                "80X reversed pair retained");
        half(words[0], 0, 0);
        require(decoder.decode(words, 2, q) == DecodeResult::Invalid, "malformed pair");
    }
    std::cout << "rotation contract passed\n";
}
