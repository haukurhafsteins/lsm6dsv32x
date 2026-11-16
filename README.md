# LSM6DSV product driver

A C++ wrapper over ST's register drivers, using `rtos::SpiDevice`. Builds select
LSM6DSV80X (default) or LSM6DSV32X. The public `imu_lsm6dsv.hpp` API contains no ST
register types. `lsm6dsv80x.hpp` remains a compatibility interface for 80X builds.

## Integration

ESP-IDF: set CMake cache variable `IMU_LSM6DSV_PART` to `80` or `32`. The component
links exactly one register layer and propagates the part define to consumers.
Invalid values fail configuration. The neutral API encodes the part into its
link symbols using an inline namespace; wrong-part consumers (including default
80X consumers linked to 32X) fail to link, without changing `imu::` call sites. Other build systems compile `lsm6dsv80x.cpp`
(the historical source-list entry point) and **one** of `lsm6dsv80x_reg.c` or
`lsm6dsv32x_reg.c`; define `IMU_LSM6DSV_PART=32` publicly for a 32X build. Do not
compile both register layers. The old 80X source list needs no changes.

Initialize the SPI device with an 8-bit command phase, then:

```cpp
imu::init_spi(&spi);
if (imu::init() != imu::InitStatus::Ready) return;
imu::Config config; // 120 Hz, +/-4 g, 2000 dps, stream FIFO
config.fifoTemperature = true; // only with a temperature-aware consumer
if (!imu::config(config)) return;
imu::start_sampling(true);
```

The driver is a single-device, externally serialized interface, as before.
`init()` reads only the build-selected identity: 0x73 for 80X, 0x70 for 32X.
Wrong identity or transport failure latches a failure: configuration, start,
restart, tapping and FIFO access perform no further transport I/O. Explicit
`init()` rechecks identity to recover. A transport failure during operation also
latches failure. The legacy void `init` wrapper retains this guard internally.
This fail-closed failure behavior is intentional; healthy 80X behavior is
checked against the frozen pre-change wrapper.

The neutral configuration supports 60/120/240/480/960 Hz, ODR-matched batching,
stream FIFO, watermark 1-255, and optional 1.875 Hz temperature batching. SFLP
runs at up to 480 Hz. Unsupported settings are rejected before I/O. The 80X
ranges are 2/4/8/16 g; 32X ranges are 4/8/16/32 g. The legacy API retains its old
configuration interpretation, including unused batching/mode/timeout fields.
Gyro bias batching remains disabled, as it was in the baseline.

## FIFO consumer contract

`fifo_read_element` returns a vendor-neutral tag, counter and six payload bytes.
These are internal driver data, not an export/log/provider payload. Keep the
consumer's existing timestamp grouping, temperature handling and drop counters.

`imu::rotation_word_count` is 2 for 80X and 1 for 32X. Supply that many
non-temperature rotation words to `RotationDecoder::decode`; it returns Ready,
NeedMore or Invalid and leaves output unchanged on failure. Extra input words
are not examined. Do not unconditionally pop a second word on 32X. Temperature
words must be handled independently by the caller, including between 80X halves.
Reset the decoder at configuration/session boundaries, dropped packets or FIFO
overrun. The caller can detect an overrun through `fifo_overrun_count()`.

80X keeps its two marker words, reversed-pair tolerance, xyzw output and
normalization. Counter equality was not enforced by the old decoder and is not
introduced here. Its legacy identity fallback for malformed pairs is preserved;
the neutral decoder instead exposes Invalid to let the consumer account for drops.

32X decodes three IEEE binary16 components x/y/z and reconstructs nonnegative
`w = sqrt(max(0, 1 - x*x - y*y - z*z))`, then normalizes. Squared norm up to 1.002
is accepted to cover binary16 rounding just outside the unit ball; larger norms
and nonfinite values are rejected. If the dot product with the previous accepted
quaternion is negative, all four components are negated. That changes quaternion
hemisphere without changing the represented rotation. Reset and invalid packets
clear continuity history. The codec does not invent samples across a discontinuity.

Accelerometer/gravity outputs are g, gyro is degrees/s, temperature is Celsius,
and timestamps are seconds. The old signed 32-bit timestamp conversion is
preserved in this compatibility change; counter unwrapping remains a caller concern.

## Validation

Use initialized `rtos` and `vectors` directories under a component directory:

`cmake -S tests -B build-host -DURU_COMPONENTS_DIR=/path/to/components`

`cmake --build build-host`

`ctest --test-dir build-host --output-on-failure`

The workflow pins both dependency commits. Tests use synthetic register/FIFO
inputs only. Frozen baseline `5990d5b` is compiled beside the new implementation;
SPI transaction values/order and derived output floats are compared exactly for
both the legacy and neutral 80X APIs. Other tests cover failure latching, scale
selection, start/stop/restart, overrun, calibration, rotation edge cases and build
selection/link contents. See [port audit](docs/32x-port-audit.md).

Host success does not prove real-board SPI timing, current consumption, FIFO
cadence, physical orientation or exercise behavior. Product integration, board
routing, physical bench checks and application validation are separate work.
No superproject pointer, release or delivery change is included here.

## Sources and licenses

The existing wrapper is MIT licensed. The unchanged vendored 32X register files
are from ST commit [dc7c1beb037dc5c61479def488084e10e8704023](https://github.com/STMicroelectronics/lsm6dsv32x-pid/tree/dc7c1beb037dc5c61479def488084e10e8704023),
release notes V4.2.0 (2026-10-08), under [BSD-3-Clause](LICENSE-ST-32X).
The 80X register files are unchanged from this repository's baseline.

Part references: [DS13511 Rev 2](https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf)
and [AN6016 Rev 2](https://www.st.com/resource/en/application_note/DM01015343-.pdf).
AN6016 section 6.4 specifies the 21.7 us nominal timestamp and signed FREQ_FINE
calibration used here; its SFLP/FIFO sections describe the 32X rotation format.
