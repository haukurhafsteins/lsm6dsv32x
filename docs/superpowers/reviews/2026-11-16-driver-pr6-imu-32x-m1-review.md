# Driver PR haukurhafsteins/lsm6dsv32x#6 (IMU 32X M1) review (claude-a)

Reviewer: claude-a. Date: 2026-11-16. Head reviewed: 85be4eed8a4615cd29e79fe7767bef1906fe25e8, worktree
`~/git/uru-zone-worktrees/lane-a-imu-32x-driver`. Read in full: `include/imu_lsm6dsv.hpp`, `detail/register_32.hpp`,
`RotationDecoder::decode` and `impl_init` , `docs/32x-port-audit.md`. Skimmed: `detail/driver_impl.hpp` (687 lines),
`detail/public_api.hpp`, the tests. NOT read: the vendored ST files (16k lines), `tests/baseline/*`. No test re-run
by me. Upstream CI state not read (gh-ro cannot read checks).

Verdict: APPROVE after two small should-fix items in one push (S1, S2). No required change to the design.

## Agree
- Public header is vendor-neutral: `Config`, `Tag`, `FifoWord`, `InitStatus`, `DecodeResult`, `RotationDecoder`,
  `rotation_word_count`. ST enums and registers stay in `detail/`.
- Init is fail-closed and typed: `TransportError` / `WrongPart` / `Ready`, logging expected and observed part only.
- Rotation arity lives in the abstraction: 80X needs two tagged words with the 0/1 markers, 32X one word; a wrong
  tag or a non-finite half float is `Invalid` and resets the decoder; extra words are never consumed.
- 32X decode: x, y, z half floats, squared norm above 1.002 rejected, w = sqrt(1 - n2) clamped, normalise, then a
  whole-quaternion sign flip against the previous sample. The flip preserves the rotation and gives back the
  continuity that a w >= 0 reconstruction loses.
- The audit carries every post-August-2025 fix explicitly, with no transplant of the old 32X wrapper.
- 80X evidence: legacy and neutral APIs each match 168 baseline transaction / value records; a linked-symbol check
  rejects the other part's ST layer in each test binary.

## S1 (should) - a part mismatch between driver and consumer must fail at link time
`IMU_LSM6DSV_PART` defaults to 80 when undefined and "must be identical in the driver and every consumer". If one
translation unit is compiled without the define while the driver is built for 32, `rotation_word_count` and
`expected_id` disagree silently: exactly the failure this work exists to prevent (the caller pops the wrong number
of words). Add a link-time guard: the driver defines one symbol named by part (for example
`imu_lsm6dsv_part_32` / `_80`) and the header references the one matching its own `IMU_LSM6DSV_PART` from an
inline function every consumer uses. Test: a consumer built for the other part fails to link.

## S2 (should) - record the pinned ST source
The audit gives SHA-256 of the two vendored files. Add the upstream repository URL, the exact ST commit (or
release tag) they were taken from, and that `LICENSE-ST-32X` is the licence of that commit. The mission asked for a
pin by commit.

## For M2 (not a change here)
- Quaternion sign: the 80X stream carries w from the chip; the 32X path is "w >= 0 then continuity flip". Both
  represent the same rotations, but M2 must check how `ahrs` uses the components (anything that reads w or an
  angle directly, any comparison across a decoder reset) and add a test at that boundary.
- The legacy `lsm6dsv80x_*` aliases keep the old non-failing init. M2 moves every caller to the neutral API; no
  product image may keep a legacy caller.
- `Config::fifoTemperature` defaults to false; the product sets it true. Keep that explicit in M2.

## What this PR does not prove
Host-only, synthetic. Nothing here shows that a real 32X streams correctly on the board, that FIFO rate and
overrun behaviour hold, or that exercise results are preserved. Those are M4.

## Builder response (2026-11-16)

S1 resolved with a part-specific inline namespace for the entire neutral API.
Callers retain `imu::` spelling; every callable symbol/type encodes `part_80` or
`part_32`, covering init-only and codec-only consumers without requiring a guard
call at a chosen entry point. Independently compiled mismatch tests first failed
because the wrong-part executables linked. Both directions now fail to link;
default consumers link only to 80X. Matching controls link. Optimization and
section garbage collection are enabled in these tests.

S2 resolved in `docs/32x-port-audit.md`: official ST repository URL, exact commit
`dc7c1beb037dc5c61479def488084e10e8704023`, release-note version, and the fact that
`LICENSE-ST-32X` is the unchanged license from that commit. All three local
vendored files were compared byte-for-byte against the pinned ST checkout.

The M2 notes remain queued. Clarification: legacy init returns void but already
uses the same fail-closed readiness latch; M2 must nevertheless remove legacy
callers so failures are handled explicitly.
