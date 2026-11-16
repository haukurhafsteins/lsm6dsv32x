# 32X port audit and host evidence

Baseline: upstream `5990d5b7f7bbee2d7a858066c0e141ae72aeb364`.
No historic 32X wrapper was restored. The common implementation was extracted
from that 80X baseline, then mapped to a pinned official 32X register layer.

## Post-August-2025 change audit

| Commits | Disposition |
| --- | --- |
| `03fd4dc` 80X adjustments | Part-specific registers/scales and rotation layout stay behind private aliases/codecs. 32X identity and 4/8/16/32 g scales come from ST. No high-g path introduced. |
| `15ae8a1` zero initialization | Preserve zero-initialized routing/SFLP structs. Initialize error-path locals before use. |
| `32d0173`, `ac08961` FIFO processing rewrite | Preserve raw-word draining, aligned decode, g/degrees-per-second units and timestamp grouping contract. The application still owns grouping. 32X uses one rotation word. |
| `5398ebb`, `7f6d09f` timestamp corrections | Share calibrated 1/(46080*(1+0.0013*FREQ_FINE)) seconds; AN6016 section 6.4 agrees. Preserve legacy signed timestamp interpretation. Positive/negative fine calibration tested. |
| `91fa5d9` enabled function | Keep the timestamp-resolution accessor enabled, using the later `7f6d09f` corrected seconds calculation rather than the intermediate ODR result. |
| `83d6fd4` README, `9948bc0` dead code removal | Current integration documentation replaces obsolete upstream-only README; dead helpers are not resurrected. |
| `290b005`, `685dce6` overrun handling | Keep count, bypass/stream resync and return 0 instead of draining torn backlog. Shared by both parts. |
| `969cb56` idle power-down | Keep bypass, SFLP disable, XL/GY ODR off. Restart restores setup before FIFO clear. Host checks register intent; current consumption needs a board. |
| `0cbfd3e` batching from sampleRate | Shared 60/120/240/480/960 mapping. Neutral API rejects higher unsupported batching rates; legacy fallbacks unchanged. |
| `e5c125b` aligned decoding | Keep memcpy to aligned locals, rather than casting six payload bytes to an aligned struct. |
| `e7775fb`, `ec1257a` RTOS transport | Keep SPI command phase, 0xFF read dummy bytes, command-only zero-length framing, RTOS logs/time. |
| `40af7aa`, `e5e05b0` FIFO temperature | Preserve explicit opt-in default false, signed little-endian decode, reconfiguration and restart/recovery behavior. |

The old 80X quaternion function has tag/marker checks but its counter check was
commented out. This PR does not silently introduce a counter-equality rejection.
The neutral result reports malformed/nonfinite inputs explicitly; the legacy
wrapper keeps the old fallback behavior. Pure decode functions remain usable on
synthetic data without hardware access.

32X reset is deliberately part-specific: the pinned ST `sw_por` routine writes
the device's software power-on-reset bit and waits 30 ms. The 80X reset sequence
and bounded polling retain their healthy-path transaction order.

## Reproducible evidence

- Before implementation, the existing FIFO-temperature test passed at the baseline.
- The new contract test initially failed to compile because `imu_lsm6dsv.hpp` did
  not exist; tests preceded the implementation.
- The baseline wrapper and header in `tests/baseline/` are byte-for-byte copies of
  that commit, with the repository's MIT license applying. They compile against
  the unchanged 80X register files. They are reference fixtures, not linked into
  a product image.
- `healthy_80x_parity` and `neutral_80x_parity` independently compare 168 exact
  transaction/derived-value records to that reference: initialization, product
  configuration, motion/gravity/temp/timestamp decode, rotation pairs in either
  order, FIFO drain/overrun, stop/restart/power-down.
- Part contracts also check wrong ID/transport latch, no I/O on failed restart,
  explicit recovery, temperature off, invalid range, 120 Hz, full-scale units,
  signed calibration, tag and counter extraction.
- Rotation tests cover identity, representative/near-pi rotations, hemisphere
  continuity/reset, rounded norm above 1, malformed/nonfinite values, truncation,
  temperature rejection and one-word versus two-word arity.
- CMake tests exercise the actual component selection and public define; a linked
  symbol check rejects the other part's ST implementation in each executable.
- All 11 CTest checks pass locally, both normally and under UBSan. Authored public-path C++ compiles with
  `-Wall -Wextra -Werror`. The frozen baseline retains its pre-existing format
  warning; the original temperature fixture deliberately tests partial aggregates.

These are driver-only synthetic tests, not superproject corpus or hardware proof.
The application must still adapt its unconditional second rotation-word pop,
reset decoder state on discontinuity, retain temperature/drop accounting, and
select the right driver by board. This PR neither builds product firmware nor
claims that physical 32X streams preserve exercise results.

Vendored 32X SHA-256:

- `lsm6dsv32x_reg.c`: `db53081c4cc61622d0f1a7fc5b457dd77b85369ba77e9ea41d287e4ffd26e0de`
- `lsm6dsv32x_reg.h`: `00cf8d4e0b6ba43819e58544f0fe831d7ca7455edb7d257b856889a1a791ff59`
