# led-lab-esp32c6

The ESP32-C6 backend for [`lp-ws281x`](../../lp-ws281x): a RISC-V implementation
of the crate's `RmtHw` trait, plus a lab firmware that drives **both** RMT TX
channels with independent chase patterns and prints a machine-checkable
pass/fail signal — **with or without strips attached**.

Phase P5 (C6 half) of the plan `2026-07-28-ws281x-rmt-driver`. TX only.

This one is a homecoming rather than a port: lp2025's
`lp-fw/fw-esp32/src/output/rmt/` is already C6 raw-register code, written by the
same author. `src/c6_rmt.rs` is that register layer cleaned up and reshaped to
fit `RmtHw`; see its module docs for what changed and why.

## What is where

| File | Contents |
|------|----------|
| `src/c6_rmt.rs` | The whole chip-specific surface: RMT RAM address, the `CH_TX_CONF0` start/stop sequence, the `INT_*` bit layout, `MEM_RADDR_EX` translation. |
| `src/main.rs` | esp-hal shell (clock, GPIO matrix, interrupt binding), the two chase patterns, the two start modes, the serial protocol. |
| `src/loopback.rs` | The `test_loopback` harness: two TX channels into two RX channels, decoded and timing-checked on the chip. |
| `../../lp-ws281x` | Everything portable: pulse encoding, ping-pong refill, the guard word, frame accounting. Tested on the host; not touched by this crate. |

## Toolchain — RISC-V, not Xtensa

The C6 core is RV32IMAC, so unlike its sibling `fw/led-lab-esp32s3` this crate
builds on **plain upstream nightly** with the `riscv32imac-unknown-none-elf`
target, not the rustup `esp` channel. Both are pinned in the crate's own
`rust-toolchain.toml` / `.cargo/config.toml`; the nightly date matches lp2025's
`lp-fw/fw-esp32`. No `-Zbuild-std` is needed — the target ships a precompiled
`core`.

One consequence worth knowing: `build.rs` here emits only `-Tlinkall.x`. The S3
crate also installs esp-hal's `--error-handling-script` linker hint, which the
RISC-V target rejects outright ("unknown argument") because it links with
`rust-lld` directly rather than through a GCC driver.

## Pins (Seeed XIAO ESP32-C6, port `/dev/cu.usbmodem11301`)

| Signal | GPIO | XIAO label | Notes |
|--------|------|------------|-------|
| Channel 0 data | 18 | **D10** | RMT `CH0`; lp2025's WS281x output header |
| Channel 1 data | 20 | **D9** | RMT `CH1` |
| Debug | 4 | *(none)* | High while channel 0's frame is on the wire; three fast pulses when a guard trips |

GPIO18 = D10 and GPIO20 = D9 are confirmed against lp2025's own board manifest,
`lp-core/lpc-hardware/boards/seeed/xiao-esp32-c6.json` — which additionally
records GPIO18 as the "known WS281x output header". That manifest is
calibration-derived rather than a datasheet transcription, so it is the best
available source short of a continuity check on the silkscreen.

GPIO4 is **not broken out** on the XIAO: the same manifest gives it no `D`
label, so the debug marker is probeable only at the module, not at a
castellation. It is unreserved there (the manifest flags GPIO12/13 as unsafe,
not GPIO4) and is the C6's MTMS pin, idle while the board is debugged over
USB-Serial-JTAG. **D3 / GPIO21 is the nearest header-accessible alternative** if
a scope probe is ever wanted; the loopback build does not use the debug pin at
all.

## The ESP32-C6 RMT RAM lives at `+0x400`

The peripheral is at `0x6000_6000` and its channel RAM at `0x6000_6400` — the
`+0x400` lp2025 hard-codes, and **half** the S3's `+0x800`. The C6 has four
48-word blocks where the S3 has eight, so the register file in front of them is
correspondingly smaller. Carrying the S3 constant over would land past the end
of RMT RAM entirely.

Verified on silicon, not just read off a table: `probe_ram_address` checks the
PAC's peripheral base, stores a sentinel through the computed pointer, then
makes the *peripheral* deposit a second sentinel through its own APB FIFO port
(`SYS_CONF.apb_fifo_mask` cleared, write to `CH<n>DATA`) and looks for it at
`RAM_BASE`. Bench result on a XIAO ESP32-C6 rev v0.2:

```text
E1: MEASURE rmt_base=0x60006000 rmt_ram=0x60006400 ram_offset=0x400 pac_base=0x60006000 ...
E1: PASS rmt_ram_offset direct=1 fifo=1 base=1
```

## Other C6-versus-S3 register differences

* **Channel split is fixed in hardware.** `CH0`/`CH1` transmit, `CH2`/`CH3`
  receive; the RX channels have no `CH_TX_CONF0`. The S3 is 4 + 4, the classic
  ESP32 lets any of its 8 channels take either role. Both S3 and C6 lay their
  blocks out contiguously from `ram_start` in channel-number order, so TX
  channel *n*'s window still starts at word *n* × 48.
* **`INT_*` interleaves TX and RX.** `ch_tx_end` 0..=1, `ch_rx_end` 2..=3,
  `ch_tx_err` 4..=5, `ch_rx_err` 6..=7, `ch_tx_thr_event` 8..=9,
  `ch_rx_thr_event` 10..=11, `ch_tx_loop` 12..=13. The *shifts* coincide with
  the S3's (0/4/8/12), which makes this easy to miss — but the TX field is two
  bits wide, not four. The S3's `0b1111` mask would clear the RX causes sitting
  next door, which esp-hal's blocking receive polls out of `INT_RAW`, and the
  loopback harness would hang instead of failing loudly.
* **`REF_CNT_RST` is not a uniform bitmask.** Its PAC fields are
  `tx_ref_cnt_rst` (CH0), `tx_ref_cnt_rst_ch1`, `rx_ref_cnt_rst_ch2`,
  `rx_ref_cnt_rst_ch3` — TX and RX interleaved again. `1 << ch` is right only
  because the two TX channels happen to be the two lowest bits.
* **`CH_TX_STATUS.mem_raddr_ex` is 9 bits** (S3: 10) — still wide enough for all
  192 words of C6 RMT RAM, which is the tell that it is an *absolute* offset
  here too, not a window-relative one. lp2025 read it raw, which was correct
  only because its single channel was channel 0. `CH_TX_LIM.tx_lim` is 9 bits on
  both chips.
* **`tx_thr_event` is not re-raised when unacknowledged.** See below — this is
  the one place the C6 behaves differently from the S3 at runtime.

## `tx_thr_event` re-raise: the C6 says no

The S3 re-raises a threshold interrupt that the handler acknowledged but whose
refill was dropped, so after a guard trip its driver's bit cursor runs one half
(24 bits) ahead of the wire. The C6 **does not**. Measured by the truncation
subtest, which suppresses exactly one threshold on channel 1:

```text
E5: MEASURE truncation ch=1 role=victim bits_rx=72 expected_stop_bits=72 ... bits_written=72 refills=1
E5: MEASURE thr_reraise chip=esp32c6 wire_bits=72 cursor_bits=72 half_bits=24 reraised=0 matches_esp32s3=0
```

`cursor_bits == wire_bits` and only the one serviced refill was counted. This is
reported, never asserted: it is a fact about silicon, and pinning it would turn
a chip revision into a test failure. The driver is correct either way — the
guard word, not the cursor, is what stops the wire, and `bits_rx` proves it
stopped in the right place on both chips.

## Serial protocol

Same `En: PASS/FAIL/MEASURE key=value` contract as the other lab firmwares.
`E1` is the RAM probe, `E2` the demo's per-channel frame accounting, `E5` the
loopback harness. The demo's headline lines:

```text
E1: PASS rmt_ram_offset direct=1 fifo=1 base=1
E2: MEASURE ch=0 leds=8 frames=30 guard_trips=0 guard_skips=0 errors=0 refill_lag_avg_words=5.5 timeouts=0
E2: MEASURE ch=1 leds=100 frames=30 guard_trips=0 guard_skips=0 errors=0 refill_lag_avg_words=6.8 timeouts=0
E2: PASS ws281x_c6_basic channels=2 frames_advancing=1 mode=simultaneous
```

A `FAIL` is emitted for a stalled frame counter, any guard trip while idle, any
`tx_err`, or a frame that misses `FRAME_TIMEOUT`.

## Memory blocks and how many channels you get

`BLOCKS_PER_CHANNEL = 1`: a 48-word window per channel, halving into 24 words =
exactly one LED, which is the tightest refill deadline the hardware can pose
(~30 µs at 800 kHz) and the only way to get **both** outputs. Raising it to 2
leaves a single output — channel 0's window would swallow channel 1's block.
`BlockPlan` validates this at compile time.

lp2025 went the other way entirely: `memsize(4)` on channel 0, which on this
chip reaches past `CH1` into the two *RX* blocks. That is sound only for a
firmware that never receives, which is why the loopback harness could not have
been built on it.

## The loopback self-test (`test_loopback`)

```bash
cd fw/led-lab-esp32c6
cargo run --release --features test_loopback
```

The feature replaces the demo loop with an on-device timing oracle
(`src/loopback.rs`): **no oscilloscope, no wires, no strips**. GPIO18 and GPIO20
are each split with esp-hal's `Flex::split()` into a frozen input/output signal
pair — the output half drives its RMT TX channel exactly as the demo does, the
input half feeds the paired RMT **RX channel** through the GPIO matrix. The
receivers capture every (level, duration) pair at the same 80 MHz / divider-1
clock, i.e. 12.5 ns resolution, with an idle threshold of 30 000 ticks (375 µs)
so a capture is ended by the post-latch idle and nothing shorter.

Both channels transmit **at once**, under different configurations:

| TX | RX | Timing | Order | LEDs |
|----|----|--------|-------|------|
| 0 | 2 | WS2812 | GRB | 2 |
| 1 | 3 | WS2812 | RGB | 1 |

Asserted per run: per-channel decode against the sent bytes; per-bit high time
and period within ±25 ns of *that channel's* pulse codes; no cross-talk; the
trailing low covers the configured 300 µs latch; a 100-frame concurrent soak
with zero mismatches, guard trips or errors; and a suppressed threshold on
channel 1 truncating it at exactly bit 72 while channel 0 finishes intact.
Verdict line: `E5: PASS loopback_esp32c6 channels=2 frames=…`.

### RX capacity

Two RX channels, one 48-word block each: 48 capture items = 48 bits = two LEDs,
because the hardware records the over-idle-threshold trailing low as a
zero-duration marker *inside* the final bit's item rather than as an extra one.
The routine captures are sized accordingly (1–2 LEDs). Longer captures work via
RX wrap (`rmt.has_rx_wrap` is true for `esp32c6`) as long as the transaction is
polled inside each 24-item (30 µs) window — the truncation subtest's 72-bit
capture relies on exactly that.

### Re-deriving the golden vector

`lp-ws281x/tests/golden/ws2812_grb_esp32c6.txt` is channel 0's GRB capture,
checked in verbatim (repo rule: golden vectors are hardware-verified, never
hand-written). To re-derive it, run the loopback as above and transcribe the
`E5: MEASURE golden_pairs` lines' `H<ticks>`/`L<ticks>` tokens into the file,
keeping the provenance header accurate (chip, date, config, frame).
`cargo test -p lp-ws281x` then validates the transcription
(`tests/golden_esp32c6.rs`): the vector must decode to the sent frame, sit
within ±25 ns of the configured timing, **and still equal the ESP32-S3 vector
tick for tick** — two RMT generations produced identical waveforms, and that
claim is asserted rather than just written down.

## The stress harness (`test_stress`, phase P6) — clean except under a WiFi scan

```bash
cd fw/led-lab-esp32c6
STRESS_SECONDS=600 cargo build --release --features test_stress
```

Both channels (300/256 LEDs) run at maximum frame rate from thread context
while an escalating load runs beside them: S0 idle, S1 verbose logging, S2 a
WiFi scan loop, S3 ESP-NOW broadcast spam, S4 STA + traffic (skipped without
`LED_LAB_WIFI_SSID` at build time — no TCP/IP stack in this firmware). The
radio stack is pulled in **only** under this feature, matching the S3 and
classic crates; version pinning note carried from the S3 crate: `esp-rtos`
**0.2 does not work** (drags in a conflicting `esp-sync`) — 0.3 does.

Result, 20 s/scenario (this chip does **not** re-raise an unacknowledged
`tx_thr_event`, so unlike the S3, `guard_trips` here is an **exact** count,
not a lower bound):

| scenario | frames | truncated | lag_max/24 | lag_over_half | verdict |
|----------|--------|-----------|------------|----------------|---------|
| S0 idle | 4 290 | **0** | 8 | 0 | clean |
| S1 logging | 676 | **0** | 7 | 0 | clean |
| S2 WiFi scan | 4 232 | **1 243 (29 %)** | 15 | 0 | trips |
| S3 ESP-NOW | 1 286 | 1 | 10 | 0 | trips (1 in 1 286) |

Zero errors, zero timeouts throughout. **Under a WiFi scan loop this chip
truncates roughly one frame in three** — a real defect, not an instrumentation
artefact (exact count, `lag_max` a comfortable 15 of 24) — while ESP-NOW,
lp2025's actual radio usage, is nearly free (1 in 1 286). That is the
project-wide recommendation from the stress phase for this chip specifically:
**raise the RMT interrupt's priority above the radio's and re-run S2 to
confirm** — RISC-V needs no assembly shim for this the way the two Xtensa
chips (S3, classic ESP32) do; it is plain priority configuration. See the
plan's `findings.md` (`2026-07-28-ws281x-rmt-driver/findings.md`, §5.2 and
§8.2) for the full per-board comparison and the go/no-go on the Xtensa
high-priority-interrupt follow-up, which this chip's numbers do not gate.

## Build and run (demo)

```bash
cd fw/led-lab-esp32c6
cargo build --release
cargo run --release            # flash the C6 and open a monitor
```

The crate declares its own `[workspace]`. `fw/` is excluded from the repo's root
workspace (different toolchain, different target), but an *ancestor* of the
checkout can still claim it — a git worktree under `.claude/worktrees/` does
exactly that — so the root is pinned here.

## Provenance

Register facts were settled from, in order: the `esp32c6` PAC 0.23.2
(`src/rmt/*.rs` — field offsets and widths), `esp-metadata-generated` 0.4.0
(`rmt.ram_start`, `rmt.channel_ram_size`, `rmt.has_rx_wrap`), and esp-hal 1.1.1's
own RMT driver (for the `mem_raddr_ex` absolute-offset convention). All
MIT/Apache-2.0. The driver structure descends from the author's own lp2025 C6
code. No GPL source (WLED, NeoPixelBus, …) was consulted; see the repo
`AGENTS.md` and `docs/adr/2026-07-28-license-provenance-discipline.md`.
