// Definition file included once by the compatibility translation unit.
namespace
{
reg::fifo_out_raw_t native_word(const imu::FifoWord &word)
{
    reg::fifo_out_raw_t native{};
    native.tag = static_cast<decltype(native.tag)>(word.tag);
    native.cnt = word.counter;
    std::memcpy(native.data, word.data.data(), word.data.size());
    return native;
}
} // namespace
namespace imu
{
void init_spi(rtos::SpiDevice *device) { impl_init_spi(device); }
InitStatus init() { return impl_init(); }
InitStatus readiness() { return status; }
bool config(const Config &value)
{
    const auto rate = static_cast<uint16_t>(value.sampleRate);
    const auto accel = static_cast<uint8_t>(value.accelRange);
    const auto gyro = static_cast<uint16_t>(value.gyroRange);
    // Reject unsupported selectors rather than silently choosing a different scale.
    if (!ready() || value.batching != Batching::FollowSampleRate ||
        value.fifoMode != FifoMode::Stream || value.fifoWatermark > 255 ||
        value.fifoWatermark == 0 ||
        (rate != 60 && rate != 120 && rate != 240 && rate != 480 && rate != 960) ||
        (accel != 4 && accel != 8 && accel != 16 && !(IMU_LSM6DSV_PART == 80 && accel == 2) &&
         !(IMU_LSM6DSV_PART == 32 && accel == 32)) ||
        (gyro != 250 && gyro != 500 && gyro != 1000 && gyro != 2000 && gyro != 4000))
        return false;
    DriverConfig next{rate,
                      accel,
                      gyro,
                      sampling_rate_to_batching(rate),
                      value.fifoWatermark,
                      reg::v_STREAM_MODE,
                      0,
                      value.fifoTemperature};
    impl_config(&next);
    return ready();
}
void start_sampling(bool start) { impl_start_sampling(start); }
void start_tapping(bool start) { impl_start_tapping(start); }
void sleep() { impl_start_sampling(false); }
void set_debug(bool enabled) { impl_set_debug(enabled); }
void read_sources(Sources &sources) { impl_read_sources(&sources); }
float get_timestamp_resolution() { return impl_get_timestamp_resolution(); }
int fifo_data_available() { return impl_fifo_data_available(); }
uint32_t fifo_overrun_count() { return impl_fifo_overrun_count(); }
int fifo_read_element(FifoWord &word)
{
    reg::fifo_out_raw_t native{};
    const auto result = impl_fifo_read_element(native);
    if (result != 0)
        return result;
    word.tag = static_cast<Tag>(native.tag);
    word.counter = native.cnt;
    std::memcpy(word.data.data(), native.data, word.data.size());
    return 0;
}
void process_xl(const FifoWord &word, Vector3<float> &value)
{
    auto n = native_word(word);
    if (driver_to_mg)
        impl_fifo_process_xl(n, value);
}
void process_gyro(const FifoWord &word, Vector3<float> &value)
{
    auto n = native_word(word);
    if (driver_to_mdps)
        impl_fifo_process_gyro(n, value);
}
void process_gravity(const FifoWord &word, Vector3<float> &value)
{
    auto n = native_word(word);
    impl_fifo_process_gravity(n, value);
}
void process_temperature(const FifoWord &word, float &value)
{
    auto n = native_word(word);
    impl_fifo_process_temperature(n, value);
}
void process_timestamp(const FifoWord &word, float &value)
{
    auto n = native_word(word);
    impl_fifo_process_timestamp(n, value);
}
DecodeResult RotationDecoder::decode(const FifoWord *words, std::size_t count, Quaternion<float> &q)
{
    if (!words || count < rotation_word_count)
        return DecodeResult::NeedMore;
    for (std::size_t i = 0; i < rotation_word_count; ++i)
        if (words[i].tag != Tag::GameRotation)
        {
            reset();
            return DecodeResult::Invalid;
        }
    Quaternion<float> next;
#if IMU_LSM6DSV_PART == 80
    const auto first = u16_le(words[0].data.data()), second = u16_le(words[1].data.data());
    if (!((first == 0 && second == 1) || (first == 1 && second == 0)))
    {
        reset();
        return DecodeResult::Invalid;
    }
    // Preserve the old decoder's marker ordering and lack of counter enforcement.
    // Reject invalid floats before the legacy decoder replaces them with identity.
    for (std::size_t i = 0; i < 2; ++i)
        for (unsigned j = 1; j < 3; ++j)
            if (!std::isfinite(npy_half_to_float(u16_le(words[i].data.data() + 2 * j))))
            {
                reset();
                return DecodeResult::Invalid;
            }
    auto a = native_word(words[0]), b = native_word(words[1]);
    impl_fifo_process_sflp_game_rotation(a, b, next);
    if (next.magnitude() == 0)
    {
        reset();
        return DecodeResult::Invalid;
    }
#else
    next.x = npy_half_to_float(u16_le(words[0].data.data()));
    next.y = npy_half_to_float(u16_le(words[0].data.data() + 2));
    next.z = npy_half_to_float(u16_le(words[0].data.data() + 4));
    const float norm2 = next.x * next.x + next.y * next.y + next.z * next.z;
    // Binary16 rounding can push a unit vector just outside the unit ball.
    // Allow <=0.002 squared-norm excess; reject corrupt finite and nonfinite data.
    if (!std::isfinite(norm2) || norm2 > 1.002f)
    {
        reset();
        return DecodeResult::Invalid;
    }
    next.w = std::sqrt(norm2 > 1.0f ? 0.0f : 1.0f - norm2);
    next.normalize();
    if (has_previous_ && next.dot(previous_) < 0)
    {
        next.x = -next.x;
        next.y = -next.y;
        next.z = -next.z;
        next.w = -next.w;
    }
    previous_ = next;
    has_previous_ = true;
#endif
    q = next;
    return DecodeResult::Ready;
}
} // namespace imu
