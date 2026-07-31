# led-lab-esp32

The **classic ESP32 (LX6)** backend for `lp-ws281x`, plus the on-device
self-tests. This is the project's deployment target, so it gets the deepest
treatment: eight RMT channels, a four-channel wire-level loopback oracle, and
an eight-channel transmit soak.

Four build modes, all from this one crate:

| Mode | Feature | What it does |
|------|---------|--------------|
| demo | *(none)* | 4 channels chasing patterns + the `E1`/`E2` serial protocol, including the on-chip RMT RAM-address probe |
| loopback | `test_loopback` | 4 TX, one RX witness per round, no wires: decodes and times the actual waveform (`E5`) |
| 8-TX soak | `soak_8tx` | all eight channels transmitting, no receivers; guard-trip counters are the signal (`E2`) |
| diagnostic | `diag` | the experiment matrix that root-caused the concurrent-receiver corruption, including a CPU `GPIO_IN` wire witness (`D5`); asserts nothing |

All four are mutually exclusive build modes — each owns the whole peripheral
and replaces the main loop. `compile_error!`s say so.

## What is where

* `src/esp32_rmt.rs` — every classic-ESP32 register fact. The seven `RmtHw`
  operations, the RAM address, the `CHnCONF1` start dance, the interrupt bit
  layout. Nothing else in the firmware knows a register name.
* `src/loopback.rs` — the `test_loopback` harness (a port of the S3's, with the
  divergences noted below).
* `src/sweep.rs` — the `sweep_channels` harness: 1..8 TX channels × two strip
  lengths, one machine-readable line per cell. Answers "how many outputs can
  this chip actually drive?".
* `src/stress.rs` — the `test_stress` harness (a port of the S3's): four
  channels of long strips while idle / logging / WiFi-scan / ESP-NOW load runs
  beside them.
* `src/main.rs` — esp-hal shell, the demo patterns, the serial protocol.

Each of `test_loopback`, `diag`, `sweep_channels` and `test_stress` replaces
`main`'s demo loop, and they are mutually exclusive (`compile_error!` says so).
`build.rs` turns "none of them is enabled" into `cfg(demo_build)` so the demo's
thirty-odd items do not each carry a growing `not(any(feature = …))` list.
* `lp-ws281x/` — all the sequencing, tested on the host.

## Pins (port `/dev/cu.usbserial-11440`)

GPIO 6–11 (flash), 0/2/12/15 (strapping) and 34–39 (input-only) are avoided.

| Signal | GPIO | Notes |
|--------|------|-------|
| Channel 0 data | 16 | RMT `CH0`; the golden vector's channel |
| Channel 1 data | 17 | RMT `CH1` |
| Channel 2 data | 18 | RMT `CH2` — WS2811 timing in both test suites |
| Channel 3 data | 19 | RMT `CH3` |
| Channel 4 data | 22 | `soak_8tx` only |
| Channel 5 data | 23 | `soak_8tx` only |
| Channel 6 data | 25 | `soak_8tx` only |
| Channel 7 data | 26 | `soak_8tx` only |
| Debug | 21 | High for channel 0's frame; three fast pulses on a guard trip |

Strips are a bonus visual check; every pass/fail signal is numeric and needs
nothing attached.

The board is a **UART bridge**, not USB-Serial-JTAG: `println!` goes out over
UART0, the port does not re-enumerate on reset, and `capture.py` must be given
an explicit baud rate (115200) because the port otherwise keeps whatever speed
the previous opener left behind.

## Build and run

```bash
cd fw/led-lab-esp32

# demo
cargo build --release
espflash flash --chip esp32 --port /dev/cu.usbserial-11440 \
  target/xtensa-esp32-none-elf/release/led-lab-esp32

# loopback self-test
cargo build --release --features test_loopback
espflash flash --chip esp32 --port /dev/cu.usbserial-11440 \
  target/xtensa-esp32-none-elf/release/led-lab-esp32

# 8-channel transmit soak
cargo build --release --features soak_8tx

# channel-count sweep (16 cells; SWEEP_SECONDS sets seconds per cell)
SWEEP_SECONDS=30 cargo build --release --features sweep_channels

# radio stress harness (STRESS_SECONDS sets seconds per scenario)
STRESS_SECONDS=600 cargo build --release --features test_stress

# capture (reset first; the port survives the reset on a UART bridge)
espflash reset --port /dev/cu.usbserial-11440
python3 ../../scripts/capture.py /dev/cu.usbserial-11440 40 115200
```

The loopback suite needs ~35 s to reach its final verdict (the 100-frame soak
and the truncation phase run after the routine assertions).

Unlike the C6, this board tolerates `espflash reset` while `capture.py` holds
the port — the UART bridge does not re-enumerate, so the simple
reset-then-capture sequence works and the `script -q /dev/null … --monitor`
workaround the C6 needs is unnecessary here.

## Where the RMT RAM is: `+0x800`, silicon-verified

`RMT_BASE + 0x800 = 0x3FF5_6800` (`esp-metadata-generated` `rmt.ram_start`
= 1073047552; PAC RMT base = `0x3FF5_6000`). Numerically the **same offset as
the ESP32-S3** — coincidence rather than shared layout, since the classic has
eight 64-word blocks (512 words) against the S3's eight 48-word ones. The C6's
`+0x400` remains the odd one out.

The demo proves it on the chip at every boot, two independent ways: a direct
store/load through the computed pointer, and a write pushed through the
peripheral's *own* address generator (`APB_CONF.apb_fifo_mask` cleared,
`CHnDATA` written) then read back at `RAM_BASE`. A wrong offset passes the
first check against any RAM and fails the second.

```
E1: MEASURE rmt_base=0x3ff56000 rmt_ram=0x3ff56800 ram_offset=0x800 \
    channel_words=64 blocks_per_channel=1 tx_channels=4 available_channels=4
E1: PASS rmt_ram_offset direct=1 fifo=1
```

## Classic-versus-S3/C6 register divergences

Each verified in esp-hal 1.1.1's `chip_specific` module for
`any(esp32, esp32s2)` (MIT/Apache-2.0), the `esp32` PAC 0.40.2 field docs, and
`esp-metadata-generated` 0.4.0. No GPL source was consulted.

| Thing | Classic ESP32 | S3 | C6 |
|-------|---------------|----|----|
| Channels | 8, each TX **or** RX (mode is per-channel config) | 4 TX + 4 RX fixed | 2 + 2 fixed |
| Block size | 64 words → 32-word halves | 48 → 24 | 48 → 24 |
| RAM offset | `+0x800` | `+0x800` | `+0x400` |
| Config regs | per-channel `CHnCONF0`/`CHnCONF1` | one `CH_TX_CONF0` | `ch_tx_conf0` |
| `conf_update` | **none** — writes take effect immediately | yes | yes |
| `INT_*` layout | interleaved *by channel*: `tx_end` = bit `3N`, `rx_end` = `3N+1`, `err` = `3N+2`, `tx_thr_event` = `24+N` | grouped by event | interleaved, 2-bit TX fields |
| Error bit | `chN_err` is a **combined TX/RX** bit; there is no separate `tx_err` | separate `tx_err` | separate |
| Wrap enable | **global** `APB_CONF.mem_tx_wrap_en` (bit 1) | per-channel | per-channel |
| Immediate TX stop | **none** (`has_tx_immediate_stop` = false) | `tx_stop` bit | `tx_stop` bit |
| `mem_owner` | exists (`CHnCONF1` bit 5) | n/a | n/a |
| Read pointer | `CHnSTATUS.mem_raddr_ex`, 10 bits, **absolute** | `CH_TX_STATUS`, absolute | `mem_raddr_ex`, 9 bits, absolute |
| `tx_lim` width | 9 bits (max 511) | — | — |
| `idle_thres` | 16 bits (`CHCONF0` bits 8:23) | 15 bits | — |

Because `INT_*` interleaves by channel, there is no contiguous per-event field
to mask in one shift the way the S3 has: `take_interrupts` tests three bits per
channel and acknowledges exactly the causes it reports. The same interleaving
is what makes the C6's warning apply here in a different shape — an S3-style
`0b1111` group mask would clear *receive* causes on this chip and hang the
loopback harness, so the backend never names a bit outside the channels it
transmits on.

### No immediate stop

`stop_tx` fills the channel's whole RAM window with end markers, exactly as
esp-hal's classic driver does; the transmitter halts at the next word boundary
(≤ 1.25 µs at WS2812 timing) and raises `tx_end`. This was carried over from a
salvaged note and **confirmed**: `rmt.has_tx_immediate_stop` is `false` and
there is no `tx_stop` field anywhere in `CHnCONF1`.

### `tx_start`, `mem_rd_rst` and `apb_mem_rst` self-clear

Measured on silicon: `CHnCONF1` reads back `0x000a0f00` before a frame, during
transmission and after completion — all three trigger bits are zero every time,
so `start_tx` setting them in one `modify` is a genuine edge on every frame and
does not need a separate clear. (`ref_always_on` and `idle_out_en` are the set
bits; `rx_filter_thres` accounts for the `0f00`.)

### `CH_TX_LIM` is a repeating count, not a window position — **and it bites**

This is the one place the portable driver's model does not survive the port,
and it cost a real bug.

The PAC says it plainly: "when channel N sends more than `tx_lim` datas then
channel N produces the relative interrupt" — a count of *entries sent* that
re-arms itself, so one fixed value fires once per that many words for the whole
frame. esp-hal's classic driver relies on exactly that, programming
`memsize.codes() / 2` **once** per transmission and never touching the register
again.

`lp-ws281x`, written against the S3 where the threshold names a *word offset
within the window*, alternates its request between the half size (32) and the
full window (64) so the event lands at each half boundary in turn. Passed
through unchanged, that asks this chip for an event every 64 words instead of
every 32 — the second refill then arrives a whole half late and the transmitter
walks into the guard word the first one planted.

Measured before the fix, demo mode:

```
E2: MEASURE ch=0 leds=8   frames=30 guard_trips=30 guard_skips=0 errors=0 refill_lag_avg_words=7.1
E2: MEASURE ch=3 leds=256 frames=30 guard_trips=30 guard_skips=0 errors=0 refill_lag_avg_words=7.0
E2: FAIL ws281x_esp32_basic reason=idle_guard_trip guard_trips_delta=120 mode=simultaneous
```

`guard_trips` exactly equal to `frames` on **every** channel whose frame
outgrew one RAM window — while `refill_lag_avg_words` sat at a comfortable 7.0
out of 32. The refills were not late; they were *asked for* late.

The backend now clamps the request to the channel's half size (`set_tx_threshold`
in `esp32_rmt.rs`), which is the only period that produces the boundary events
the core expects. After:

```
E2: MEASURE ch=0 leds=8   frames=30 guard_trips=0 guard_skips=0 errors=0 refill_lag_avg_words=5.2
E2: MEASURE ch=3 leds=256 frames=30 guard_trips=0 guard_skips=0 errors=0 refill_lag_avg_words=7.9
E2: PASS ws281x_esp32_basic channels=4 frames_advancing=1 mode=simultaneous
```

A request *smaller* than a half is passed through untouched, so the test hook
that suppresses a threshold still behaves as written — the truncation test
still stops the victim at exactly bit 96.

**Backport note:** this is a chip-semantics difference, not a core bug, and it
is handled entirely in the backend. If `lp-ws281x` ever grows a non-uniform
block plan, the clamp needs the per-channel half rather than the plan's.

## `tx_thr_event` re-raise: the classic says **no**

The S3 re-raises an unacknowledged `tx_thr_event`, so its bit cursor can run one
half ahead of the wire after a suppressed threshold. The C6 does not. Measured
here, with the truncation test's threshold suppressed on channel 2:

```
E5: MEASURE thr_reraise bits_written=96 expected_stop_bits=96 delta=0 reraises_like_s3=0
E5: MEASURE truncation ch=2 role=victim bits_rx=96 expected_stop_bits=96 total_bits=384 \
    prefix_ok=1 guard_trips_delta=1 bits_written=96 refills=1
```

`delta=0`: the driver wrote exactly the 96 bits the wire carried (one 64-word
prefill plus the single refill that *was* serviced), with no phantom extra
refill. **The classic behaves like the C6, not like the S3** — the re-raise is
an S3-specific quirk. Consequence, same as on the C6: `guard_trips` is an exact
count here, where on the S3 it is a lower bound.

## Serial protocol

`E1` — RAM probe (demo only). `E2` — demo/soak counters. `E5` — loopback.
`MEASURE` lines carry numbers, `PASS`/`FAIL` carry verdicts. The final verdict
repeats every 2 s so any capture window catches it.

## Memory blocks and how many channels you get

Eight 64-word blocks. `BLOCKS_PER_CHANNEL = 1` gives all eight channels a
window, halving to 32 words = 1⅓ LEDs — the tightest refill deadline this chip
poses (~40 µs at 800 kHz). Measured refill lag is 5.2–7.9 words of the 32
available, so roughly a quarter of the budget is used at four channels and up
to a quarter at eight.

## The loopback self-test (`test_loopback`)

Four TX channels (GPIO16–19) routed into four RMT **RX** channels through the
GPIO matrix with `Flex::split` — no wires, no strips. Every (level, duration)
pair is captured at 12.5 ns and the wire protocol is asserted numerically while
all four channels transmit at once, under four different configurations
(WS2812/GRB, WS2812/RGB, **WS2811**, WS2812/BGR).

All four channels transmit every round; **one** receiver is armed per round and
the rounds rotate. That is not a reduction in coverage — it is what makes the
measurement valid at all. See "Root cause" below.

Current state on the attached board, two fresh boots, identical output:

| Assertion | Result |
|-----------|--------|
| `loopback_decode` ch0–3 | PASS |
| `loopback_timing` ch0–3 (±2 ticks) | PASS — every bit *exactly* nominal, zero spread |
| `loopback_latch` ch0–3 | PASS |
| `loopback_cross_talk` | PASS |
| `loopback_truncation` | PASS — victim stopped at exactly bit 96, bystanders clean |
| `loopback_soak` (100 witnessed frames per channel, 4 transmitting) | PASS — 0 mismatches |

Final line: `E5: PASS loopback_esp32 channels=4 frames=405` — 15 `PASS` lines,
identical on two fresh boots. (This soak was red until 2026-07-29; the cause
was the harness, see below.)

Measured timings, all four channels, both boots:

```
ch0/1/3 (WS2812): t0h 32..32  t1h 64..64  period 100..100   (nominal 32 / 64 / 100)
ch2     (WS2811): t0h 24..24  t1h 72..72  period 100..100   (nominal 24 / 72 / 100)
```

### RX capacity — the classic constraint

`rmt.has_rx_wrap` is **false**: a receiver stops at the end of its window and
esp-hal rejects a capture buffer larger than it. With four TX + four RX
channels at one block each, a routine capture is at most 64 items = 64 bits.
The 1–2 LED test frames fit; the truncation victim (which must reach bit 96)
does not, so the truncation phase reconfigures the peripheral: RX ch4 takes two
blocks (absorbing ch5's) and watches the victim, ch6/ch7 keep one block each
and watch two bystanders, and TX ch3 runs unobserved with its driver counters
as the assertion. That is the one structural divergence from the S3 harness,
where RX wrap let a 48-item window fill a 96-item buffer.

### The RX input filter is **not** optional here

The S3 and C6 harnesses run with the receiver's input filter off. On the
classic that costs real captures: four adjacent GPIOs switching in lockstep
inject simultaneous-switching glitches, and a wrap-less receiver with no filter
records each one as an extra edge. The harness therefore sets
`filter_threshold = 15` ticks (187 ns) — below the shortest legitimate high
time in the suite (WS2811's T0H = 300 ns = 24 ticks) and far above the
glitches.

Evidence: a diagnostic phase put **two** RX channels on one wire (GPIO18) and
compared them. With the filter off, the two witnesses agreed with each other
50/50 but matched the expected bytes only 46/50. With the filter on, 50/50 on
both. Captures with a spurious 49th item on a 48-bit frame stopped appearing.

Caveat worth recording: that experiment ran with *two* receivers armed, which
the next section shows is itself a perturbing configuration. The two findings
are distinguishable by symptom — a spurious **extra item** (an edge that was
never transmitted) versus a **substituted word** with another channel's exact
pulse width — and only the filter fixes the first. The filter is kept.
`golden_esp32.rs` asserts that every pulse in the checked-in vector clears the
filter by a wide margin, so the filter can never be silently measuring itself.

### Root cause: **two or more concurrent RMT RX channels corrupt the transmitters' own output**

For a while this suite's 100-frame soak reported mismatches on ~5-8 % of
channel-frames, single-bit, with the bad bit carrying *another channel's* pulse
width at the same word index. It is root-caused, and the transmitters were
never at fault. The mechanism is a property of the classic ESP32's RMT, and it
is triggered by the **harness**, not by the driver:

> Whenever **two or more RMT RX channels are capturing at the same time**, a
> concurrently transmitting RMT TX channel can put a word belonging to a
> *different* channel on its pad. One receiver — or none — and the wire is
> exact.

That is a real corruption of the pad, not a mis-capture. It was proved by an
instrument that is not the RMT receiver: a `#[ram]` CPU loop that
edge-timestamps `GPIO_IN` with `CCOUNT` (~42 cycles = 175 ns per sample, ample
to separate a 400 ns high from an 800 ns one) while the RMT RX channel captures
the *same* frame. Over 400 witnessed frames per configuration:

| receivers armed | frames witnessed | both witnesses called it corrupt | **one saw it, the other did not** | bits wrong / bits checked (CPU) |
|---|---|---|---|---|
| 4 | 400 | 21 | **0** | 66 / 8 645 |
| 1 | 400 | 0 | **0** | 0 / 8 776 |
| 0 | 400 | 0 | **0** | 0 / 8 776 |

`rx_only_bad = cpu_only_bad = 0` in every block: there is not one frame in 1 200
where the RMT receiver and the CPU disagreed about whether the pad was right.
On 16 of the 21 corrupt frames they also named the *same* first bad bit; the
other five carry several substituted words, and the CPU's ±14-tick
quantisation does not always flip the same one first.

Both witnesses are judged over exactly the same bit range, and the CPU witness
is aligned on the **end** of the frame (it always runs to the terminating idle;
where it starts depends on where the caller's code landed in flash), so a
"clean" CPU verdict can never be a blind one. The measured widths say the same
thing independently: with ≤ 1 receiver the CPU witness reports every high as
one of exactly two quantised values, and only with ≥ 2 receivers does a third
band appear carrying the other channels' WS2811 timing.

The rest of the matrix (`--features diag`, `src/diag.rs`, 100 frames per block)
locates it precisely:

```
label              tx  rx   misses per wire (w0/w1/w2/w3)
x1_baseline         4   4   7 / 19 / 2 / 1      <- the original red
x2_rx_only_w0..w3   4   1   0 / 0 / 0 / 0       <- 4x100 frames, zero
x3_tx1..tx4       1-4   1   0 / 0 / 0 / 0       <- transmitter count is irrelevant
x2b_rx45            4   2   0 / 15 / 0 / 0      <- two receivers is enough
x2b_rx46            4   2   7 / 0 / 0 / 0
x2c_no_rx4..rx7     4   3   up to 7 / 19 / 2 / 0
x7_spread           4   4   7 / 19 / 2 / 1      <- GPIO 16/22/25/27, not 16-19
x8_stagger16/50/137 4   4   18/0/15/2, 29/0/18/4, 9/1/0/2  <- never removed
x6_ram_scan         4   4   ram_foreign=0       <- RAM *contents* are never wrong
```

What each line rules out:

* **Not transmitter concurrency.** One fixed witness with 1, 2, 3 and 4
  concurrent transmitters: zero misses in all four blocks, including frames
  that refill mid-transmission (`x3_tx4` runs the same `[2,1,2,1]` LED lengths
  as the soak, so channels 0 and 2 each take a refill).
* **Not pad coupling (H1).** Moving the four data pins from adjacent
  GPIO16/17/18/19 to spread GPIO16/22/25/27 changes the miss counts by ≤ 1.
  (The separate RX-filter finding below is a genuine pad-level effect, and it
  is a different one.)
* **Not the start phase (H4).** Staggering the starts by 16, 50 or 137 ticks
  moves the misses between channels but never removes them.
* **Not RMT RAM corruption.** Scanning all four transmit windows after every
  frame for words outside that channel's own codebook: `ram_foreign=0` in every
  block. The stored words are right; it is the *fetch* that delivers the wrong
  one. (This is the check the earlier stamped-window test could not make: that
  one ran with no receivers armed, i.e. in the configuration that never fails.)
* **Not the receiver's capture path.** The CPU witness above never touches RMT
  RAM, the RX input filter, or the receive state machine, and it sees the same
  wrong pulse at the same index.

Mechanistically this is consistent with a read-during-write collision on the
shared 512-word RMT RAM: the transmitter's fetch returns the data of a
receiver's concurrent write. It needs *two* writers to bite, which is why one
armed receiver is clean. That last step is inference; the measurements above
are not.

**What this means for the deployment target.** Production transmits and does
not receive, so this cannot occur there: 4 concurrent transmitters with zero
receivers were witnessed clean on the pad for 400 frames per wire. The bug was
in the *instrument*.

**The fix (a fix, not a workaround).** Both loopback phases now arm **one
receiver per round** while all four channels keep transmitting. Coverage went
up rather than down — the soak now witnesses `SOAK_FRAMES` frames per channel
out of `4 x SOAK_FRAMES` transmitted, where it used to witness 100 total — and
no assertion or tolerance was relaxed. Result on two fresh boots:

```
E5: MEASURE soak ch=0 witnessed_frames=100 tx_frames=400 mismatches=0 guard_trips=0 guard_skips=0 errors=0 refill_lag_avg_words=2.5 refills=400
E5: MEASURE soak ch=1 witnessed_frames=100 tx_frames=400 mismatches=0 ...
E5: MEASURE soak ch=2 witnessed_frames=100 tx_frames=400 mismatches=0 ... refills=400
E5: MEASURE soak ch=3 witnessed_frames=100 tx_frames=400 mismatches=0 ...
E5: PASS loopback_soak witnessed_frames=100 channels=4 tx_concurrent=4 rx_armed=1
E5: PASS loopback_esp32 channels=4 frames=405
```

The golden vector is byte-identical to the one captured under the old
four-receiver configuration, so the checked-in file did not need re-deriving.

**Residual exposure, deliberately left.** The truncation phase still arms
**three** receivers at once (it observes the victim and two bystanders in the
round where the threshold is suppressed, and the suppression hook is
one-shot). It passes deterministically on every boot recorded so far — the
frames there are fixed, and the corruption is deterministic for a given data
pattern — but it is running in the configuration that can corrupt. Making it
robust means running it three times with the hook re-armed and one receiver
each; that was not done here to avoid churning a green test during a
diagnosis.

**Reproducing the corruption on purpose.** `cargo build --release --features
diag` and flash. It prints `D5: MEASURE` lines only, asserts nothing, and
takes ~60 s. `src/diag.rs` documents each experiment.


### Re-deriving the golden vector

1. Flash the `test_loopback` build and capture the run.
2. Transcribe the `E5: MEASURE golden_pairs` lines (channel 0's capture,
   in order) into `lp-ws281x/tests/golden/ws2812_grb_esp32.txt`, eight `H/L`
   pairs per line, keeping the provenance header current.
3. `cargo test -p lp-ws281x` — `golden_esp32.rs` re-checks the decode, the
   ±2-tick tolerance, the filter margin, and the cross-chip equality.

The classic capture is **byte-for-byte identical** to the S3's and the C6's:
three RMT generations, three channel counts, three block sizes, one waveform.
`the_classic_s3_and_c6_captures_are_the_same_waveform` asserts it so the claim
cannot rot.

## The 8-TX soak (`soak_8tx`)

All eight channels transmit (8/16/100/256/32/64/150/200 LEDs), no receivers, so
the driver's counters are the only oracle. Result on the attached board, ~1000
frames per channel:

```
E1: PASS rmt_ram_offset direct=1 fifo=1        (tx_channels=8, available_channels=8)
E2: MEASURE ch=0 leds=8   frames=1189 guard_trips=0   guard_skips=0   errors=0 refill_lag_avg_words=5.6 timeouts=0
E2: MEASURE ch=1 leds=16  frames=992  guard_trips=0   guard_skips=0   errors=0 refill_lag_avg_words=6.7 timeouts=0
E2: MEASURE ch=2 leds=100 frames=876  guard_trips=2   guard_skips=2   errors=0 refill_lag_avg_words=7.5 timeouts=0
E2: MEASURE ch=3 leds=256 frames=825  guard_trips=23  guard_skips=21  errors=0 refill_lag_avg_words=7.6 timeouts=0
E2: MEASURE ch=4 leds=32  frames=1110 guard_trips=8   guard_skips=5   errors=0 refill_lag_avg_words=6.7 timeouts=0
E2: MEASURE ch=5 leds=64  frames=903  guard_trips=439 guard_skips=321 errors=0 refill_lag_avg_words=6.8 timeouts=0
E2: MEASURE ch=6 leds=150 frames=849  guard_trips=88  guard_skips=75  errors=0 refill_lag_avg_words=7.2 timeouts=0
E2: MEASURE ch=7 leds=200 frames=772  guard_trips=193 guard_skips=145 errors=0 refill_lag_avg_words=7.7 timeouts=0
E2: FAIL ws281x_esp32_soak8 reason=idle_guard_trip guard_trips_delta=13 mode=free_running
```

This is a **different** open item from the concurrent-receiver corruption
above, and the two do not interact: this build arms no receivers at all, which
is exactly the configuration the CPU wire witness found clean. What is open
here is refill *starvation*, not data corruption — and it has no wire oracle,
because eight transmitters leave no channel free to receive.

Every channel keeps advancing, no `errors`, no `timeouts` — but guard trips
accumulate on the longer strips, worst on channel 5 (roughly half its frames).
`guard_skips` tracks `guard_trips` almost one-for-one, which says the refill is
arriving with the read pointer sitting *on* the guard slot rather than past it.
Mean refill lag stays at 5.6–7.7 words of the 32 available, so this is not a
plain deadline overrun. Four channels are clean here (demo mode, `E2: PASS`);
eight are not. Same caveat as the soak above: reported, not tuned away.

## The channel-count sweep (`sweep_channels`) — and what it caught

`soak_8tx` above left the question "is eight channels too many?" open, because
it varied channel count, strip length and phase all at once. `sweep_channels`
varies one thing: it walks 1..8 transmitters with **the same** strip length on
every active channel, started together, two lengths (100 and 300 LEDs), one
`E7: CELL` line per cell. `SWEEP_SECONDS` sets seconds per cell (default 30).

### Read `refills` vs `refills_wanted`, not `lag_max`

The trap the earlier soak fell into: `refill_lag` measures how far the read
pointer moved **while a refill ran** — the handler's own cost — not how late
the refill was. A refill that never arrives leaves no lag sample at all, so a
channel can truncate every frame with `lag_max` sitting at a quarter of the
32-word deadline. A frame of `leds` LEDs needs `leds * 24 / 32` refills; the
sweep prints that as `refills_wanted` beside the `refills` that happened.

### What the sweep found: ROOT-CAUSED — an ISR throughput ceiling, not a phase bug

Baseline, 30 s per cell:

```
ch leds  frames trunc  trunc% lag_max half refills   wanted    got%  irq_hz demand
 1  100    8998     0    0.0%      14   32   674850   674850 100.0%   22495  22495
 2  100   17878     0    0.0%      11   32  1340850  1340850 100.0%   44695  44695
 3  100   26496  8832   33.3%      12   32  1413117  1987200  71.1%   47103  66240
 4  100   28704 14352   50.0%      11   32  1349073  2152800  62.7%   44969  71760
 1  300    3215     0    0.0%      11   32   723375   723375 100.0%   24112  24112
 2  300    6414     0    0.0%      11   32  1443150  1443150 100.0%   48105  48105
 3  300    9579  3193   33.3%      12   32  1468780  2155275  68.1%   48959  71842
```

Truncation starts at **three** channels, at both lengths, and an earlier pass
at this file treated it as an open phase-dependent mystery: `lag_max` never
climbs (never past 14 of 32, `lag_over_half` always 0), so the refills that
*do* run are comfortable, and the failing channels stop after an exact,
unvarying per-frame refill count rather than a noisy one. That reading is
superseded — the column that actually carries the signal is `irq_hz` versus
the demand:

```
ch leds  irq_hz delivered   irq_hz demanded   trips
 1  100          22 487            22 487         0
 2  100          44 350            44 350         0
 3  100          48 720            66 450     1 773
 4  100          46 340            88 550     5 313
 8  300          55 313           191 400     3 828
```

**The delivered interrupt rate flatlines at ~46 000–55 000/s no matter how
much is demanded.** A continuously-transmitting channel demands
`800_000 / half_words` refills/s — 25 000/s at these 32-word halves — so two
channels demand exactly 50 000/s, which is where the ceiling sits. That is why
the boundary is at two channels and why it is sharp: below the ceiling every
channel gets everything it asks for (100 % delivered, 0 trips); at or above it,
the channels that lose the race stop asking (their trips stop the frame), which
is why the survivors' `lag_max` stays low — the aggregate rate barely moves
because the failing channels are no longer part of the demand.

**Stagger was tested as the leading hypothesis and does not fix it.**
`SWEEP_STAGGER_TICKS=1600` (half a half-period) against the 0 control, same
binary otherwise:

```
cell          control trips/complete   staggered trips/complete
3 ch x 100         1 773 / 3 543            1 605 / 3 210
4 ch x 100         5 313 / 1 771            3 210 / 3 210
3 ch x 300           640 / 1 280              616 / 1 232
4 ch x 300         1 916 /   640            1 232 / 1 232
```

Staggering improves the 4-channel completion ratio (25 % → 50 %) but
eliminates nothing, and `irq_hz` stays pinned at ~46–48 k throughout.
Coincident deadlines are a real secondary effect — they are what makes
equal-length, simultaneously-started channels the worst case — but they are
not the cause. Two earlier `set_tx_threshold` rewrites (caching the write,
with and without a per-frame re-arm) were also tried on silicon and are
recorded as negative results in that function: both *moved* the failure
between channels without removing it, which is consistent with a throughput
ceiling and not with a per-channel bug in the threshold write.

**The consequence follows from the arithmetic.** Max simultaneous channels ≈
`ceiling × half_words / 800_000` ≈ `half_words / 16.7`:

| blocks/channel | half | max channels (arithmetic) |
|---|---|---|
| 1 (32 words, shipped) | 32 | **2**, measured exactly |
| 2 (64 words) | 64 | ~4 — marginal, 50 000/s demand against a ~48 000/s ceiling |
| 4 (128 words) | 128 | ~8 channels of headroom, but only 2 outputs fit that many blocks |

So a 4-output classic product wants **2 blocks per channel**, landing right at
the edge of the measured ceiling — which is where the high-priority-interrupt
follow-up stops being only a WiFi-robustness question (see the stress harness
below) and becomes a *capacity* question too: a faster, less-preemptible ISR
raises the ceiling that gates channel count. The 2-blocks-per-channel
configuration (`BLOCKS_PER_CHANNEL` env override here, `SLOT_STRIDE`/
`USABLE_CHANNELS` in `sweep.rs`) is implemented but does not yet build in the
sweep harness — it still asks esp-hal to configure the blocks a wider channel
absorbed. The ~4-channel figure above is therefore arithmetic, not a
measurement, until that plumbing is fixed.

## The stress harness (`test_stress`) — radio load *does* break the refill

Four channels of unequal length (300/256/200/150) on GPIO16-19 at the maximum
frame rate, with escalating load beside them. `STRESS_SECONDS` sets seconds per
scenario. The image is 488 KB against a 4 MB part, so **no custom partition
table is needed** despite the radio blob.

| scenario | load | frames | truncated | errors | timeouts | lag_max/32 | ≥half |
|----------|------|--------|-----------|--------|----------|------------|-------|
| S0 | idle | 64 984 | 4 (0.006 %) | 0 | 0 | 15 | 0 |
| S1 | logging | 4 556 | 0 | 0 | 0 | 11 | 0 |
| S2 | WiFi scan loop | 80 328 | **53 402 (66 %)** | 0 | 0 | **34** | 4 |
| S3 | ESP-NOW broadcast | 20 288 | **10 145 (50 %)** | 0 | 0 | 16 | 0 |
| S4 | STA + traffic | not run — no AP configured | | | | | |

S0 and S1 are clean, so the driver itself is sound at this fan-out. **The radio
is not.** S2 is the worst case and it is the only measurement anywhere in this
firmware where a refill has actually **overrun the deadline**: `lag_max` = 34
against a half of 32, with four refills landing in the final histogram bucket.
That is a genuine deadline overrun, not the "refill never arrived" signature
the sweep produces — the WiFi stack is holding the CPU (or interrupts) long
enough that a level-3 handler misses its window.

S3 matters most, because ESP-NOW broadcast is lp2025's actual radio usage: half
the frames truncate, and channel 1 truncates *every* frame. `lag_max` stays
inside the deadline there, so S3's damage is the scarcity signature rather than
the lateness one.

Note S1's frame count: verbose logging drops the frame rate by 14x (the burst
blocks for 50 ms) but never truncates. Serial chatter costs throughput, not
correctness — the opposite of what the lp2025 comment assumed.

**The conclusion for a shipping decision:** on the classic ESP32, WS2812 output
at full frame rate and a live radio do not currently coexist. Whether the fix
is the level-4/5 assembly shim, a lower frame rate, or bigger RMT windows (two
blocks per channel halves the interrupt rate at the cost of four outputs) is
the follow-up question this phase was meant to raise.

## Interrupt teardown before releasing the peripheral

A phase that drops its esp-hal `Channel` handles takes the RMT's clock away
with the last one. An interrupt arriving after that point runs the driver's
handler against a clock-gated peripheral, which stalls the bus access and
wedges the CPU inside a maximum-priority handler — no output, no reset, no way
out.

The loopback harness hit this **twice** and hung silently both times: once
leaving the main phase (so the truncation phase never ran at all — every early
run stopped dead after `loopback_soak`), and once leaving the truncation phase
(so the suite reached its verdict and then wedged before printing it). Both are
fixed by calling `esp32_rmt::disable_all_interrupts()` before the handles go
out of scope. Anything that tears an `Rmt` down on this chip needs the same.

## Provenance

Register facts derived, in the order they settled each question, from esp-hal
1.1.1 `src/rmt.rs` (`chip_specific` for `any(esp32, esp32s2)`, MIT/Apache-2.0),
the `esp32` PAC 0.40.2 field docs, and `esp-metadata-generated` 0.4.0 `rmt.*`
properties. Silicon confirmed the RAM offset, the trigger-bit self-clearing,
the `tx_lim` semantics and the `tx_thr_event` non-re-raise. **No WLED or
NeoPixelBus source was consulted** (GPL).
