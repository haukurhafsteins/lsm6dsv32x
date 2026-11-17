"""Link independently compiled consumers against each selected driver archive."""
import pathlib
import subprocess
import shlex
import sys
import tempfile

compiler, driver_part, archive, components, include_dir, *flag_strings = sys.argv[1:]
flags = [flag for value in flag_strings for flag in shlex.split(value)]
source = r'''
#include "imu_lsm6dsv.hpp"
#include "rtos/Log.hpp"
#include "rtos/time.hpp"
namespace rtos {
bool SpiDevice::transfer_cmd(uint16_t, const uint8_t*, uint8_t*, size_t) { return false; }
void Log::log(LogLevel, const char*, const char*, ...) {}
}
namespace rtos::time { void sleep_for(Millis) noexcept {} }
int main() {
#ifdef CODEC_ONLY
    imu::RotationDecoder decoder;
    imu::FifoWord words[imu::rotation_word_count]{};
    Quaternion<float> q;
    return static_cast<int>(decoder.decode(words, imu::rotation_word_count, q));
#else
    return static_cast<int>(imu::init());
#endif
}
'''
with tempfile.TemporaryDirectory(prefix='imu-part-link-') as tmp:
    root = pathlib.Path(tmp)
    consumer = root / 'consumer.cpp'
    consumer.write_text(source)
    for codec_only in (False, True):
        for consumer_part in ('80', '32', 'default'):
            obj = root / 'consumer.o'
            command = [compiler, '-std=c++20', '-O2', '-ffunction-sections', '-fdata-sections',
                       '-I' + include_dir, '-I' + components + '/rtos/include',
                       '-I' + components + '/vectors/include']
            if consumer_part != 'default':
                command += ['-DIMU_LSM6DSV_PART=' + consumer_part]
            if codec_only:
                command += ['-DCODEC_ONLY=1']
            subprocess.run(command + flags + ['-c', str(consumer), '-o', str(obj)], check=True)
            linked = subprocess.run([compiler, *flags, str(obj), archive, '-Wl,--gc-sections',
                                     '-o', str(root / 'consumer')], capture_output=True, text=True)
            effective_part = '80' if consumer_part == 'default' else consumer_part
            should_link = effective_part == driver_part
            assert (linked.returncode == 0) == should_link, (
                f'driver={driver_part}, consumer={consumer_part}, codec_only={codec_only}: '
                f'expected link={should_link}, got={linked.returncode}\n{linked.stderr}')
            if not should_link:
                assert 'undefined reference' in linked.stderr and 'imu::' in linked.stderr, linked.stderr
print(f'{driver_part}X: matching consumers link; wrong/default-other-part consumers fail')
