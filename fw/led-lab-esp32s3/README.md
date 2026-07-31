# led-lab-esp32s3

The first real-hardware backend for [`lp-ws281x`](../../lp-ws281x): an ESP32-S3
implementation of the crate's `RmtHw` trait, plus a lab firmware that drives
**all four** RMT TX channels with independent chase patterns and prints a
machine-checkable pass/fail signal — **with or without strips attached**.

Phases P2 (backend + demo), P3 (loopback oracle) and P4 (four concurrent
channels) of the plan `2026-07-28-ws281x-rmt-driver`. TX only; other chips are
P5.

## What is where

| File | Contents |
|------|----------|
| `src/s3_rmt.rs` | The whole chip-specific surface: RMT RAM address, the `CH_TX_CONF0` start/stop sequence, the `INT_*` bit layout, `MEM_RADDR_EX` translation. |
| `src/main.rs` | esp-hal shell (clock, GPIO matrix, interrupt binding), the four chase patterns, the two start modes, the serial protocol. |
| `src/loopback.rs` | The `test_loopback` harness: four TX channels into four RX channels, decoded and timing-checked on the chip. |
| `../../lp-ws281x` | Everything portable: pulse encoding, ping-pong refill, the guard word, frame accounting. Tested on the host; not touched by this crate. |

## Pins

| Signal | GPIO | Notes |
|--------|------|-------|
| Channel 0 data | **4** | RMT `CH0` routed through the GPIO matrix by esp-hal. |
| Channel 1 data | **5** | RMT `CH1`. |
| Channel 2 data | **6** | RMT `CH2`. |
| Channel 3 data | **7** | RMT `CH3`. |
| Debug | **15** | High for the duration of channel 0's frame; three fast pulses when a guard trip is observed. |

All are proposed defaults — nothing on the board dictates them. GPIO 19/20 are
the USB-Serial-JTAG data lines and must stay clear; 4–7 and 15 are otherwise
uncommitted.

Strips on GPIO 4–7 are a bonus visual check; the pass/fail signal below does not
depend on them. The four demo strips have different lengths (8, 16, 100, 256
LEDs) and different wire configurations, so four strips side by side are told
apart at a glance.

## The ESP32-S3 RMT RAM lives at `+0x800`

`RMT_BASE` is `0x6001_6000` and the channel RAM starts at **`0x6001_6800`** —
*not* the `+0x400` the ESP32-C6 uses. The S3 has eight 48-word blocks against
the C6's four, and a correspondingly larger register file in front of them.
Carrying the C6 constant over lands in the tail of the register block: the
driver would appear to work while the transmitter sent whatever RAM happened to
hold.

The constant is taken from `esp-metadata-generated` 0.4.0 (`rmt.ram_start` for
`esp32s3` = `1610704896` = `0x6001_6800`; MIT/Apache-2.0 — the same table
esp-hal's own RMT driver reads), and is re-checked on the chip at every boot by
`s3_rmt::probe_ram_address`, which

1. stores a sentinel through the computed pointer and reads it back, and
2. clears `SYS_CONF.apb_fifo_mask` (`0` = "access memory by FIFO"), writes a
   second sentinel to `CH0DATA`, and looks for it at the computed address.

Step 2 is the one that matters: that word is placed by the *peripheral's own*
address generator, so finding it where the constant says proves the constant.
Step 1 alone would pass against any writable memory.

## Other S3-versus-C6 register differences

* `INT_RAW`/`INT_ST`/`INT_ENA`/`INT_CLR` are grouped **by event, then channel**:
  `tx_end` 0..=3, `tx_err` 4..=7, `tx_thr_event` 8..=11, `tx_loop` 12..=15, then
  the RX events. The C6 interleaves TX and RX (`tx_end` 0..=1, `rx_end` 2..=3,
  …). The PAC accessor *names* are identical on both chips, so a hand-rolled
  mask ports silently and wrongly.
* `CH_TX_STATUS.mem_raddr_ex` is a 10-bit offset into the **whole** 384-word RMT
  RAM, not into the channel's window; the backend subtracts the channel's window
  start (`ch * 48` — `BlockPlan::window_start`).
* `CH_TX_LIM.tx_lim` is 9 bits (max 511).
* An **unacknowledged `tx_thr_event` is re-raised**. Found by P4's truncation
  test: when the driver's test hook deliberately swallows a threshold cause, the
  peripheral raises it again, and the replacement refill is serviced in the
  window between the transmitter latching the guard word and the `tx_end` that
  reports it. Harmless on the wire (it writes RAM nobody reads) but it advances
  the refill cursor — see the caveat in the core's README.

## Serial protocol

`E1` covers bring-up, `E2` the running driver, `E4` the loopback self-test.

```
led-lab-esp32s3: ws281x RMT driver, 4 channels on GPIO4-7 (8 LEDs), debug on GPIO15
E1: MEASURE rmt_base=0x60016000 rmt_ram=0x60016800 ram_offset=0x800 channel_words=48 blocks_per_channel=1 tx_channels=4 available_channels=4
E1: PASS rmt_ram_offset direct=1 fifo=1
E2: MEASURE ch=0 leds=8 frames=30 guard_trips=0 guard_skips=28 errors=0 refill_lag_avg_words=4.1 timeouts=0
E2: MEASURE ch=1 leds=16 frames=30 guard_trips=0 guard_skips=1 errors=0 refill_lag_avg_words=4.5 timeouts=0
E2: MEASURE ch=2 leds=100 frames=30 guard_trips=0 guard_skips=21 errors=0 refill_lag_avg_words=4.9 timeouts=0
E2: MEASURE ch=3 leds=256 frames=30 guard_trips=0 guard_skips=0 errors=0 refill_lag_avg_words=4.9 timeouts=0
E2: PASS ws281x_s3_basic channels=4 frames_advancing=1 mode=simultaneous
```

The `E2` block repeats once a second, one `MEASURE` per channel and one verdict
for the set. A `FAIL` replaces the `PASS` when, over the last second, any
channel stopped advancing (`reason=frames_stalled`), the guard word truncated a
frame (`reason=idle_guard_trip`), the transmitter reported `tx_err`
(`reason=tx_err`), or a frame did not finish within 50 ms
(`reason=frame_timeout`).

`mode=` names the start mode, which alternates every 5 s:

* `simultaneous` — all four frames start within a few register writes of each
  other, so the channels cross their half boundaries in lockstep and the handler
  is entered with up to four coincident thresholds;
* `free_running` — each channel restarts on its own interval (17/23/29/33 ms),
  so refills arrive at unrelated phases and one channel's `tx_end` routinely
  shares a snapshot with another's refill.

`guard_trips` is the load-bearing number. Nothing else runs on this firmware —
no WiFi, no other peripheral — so the refill interrupt has the CPU to itself and
a trip here is a driver bug rather than a symptom of contention. `guard_skips`
is *not* a failure: it counts refills that declined to plant a guard because the
read pointer had not yet passed the slot, which is expected at the wrap
boundary.

`guard_skips` is expected to be non-zero here, and per-channel: the handler
serves channels in index order, so the first channel of a coincident group is
often entered before its read pointer has left the guard slot. Measured on this
board with four channels busy: ~1 skip per frame on channel 0, none on
channel 3.

`refill_lag_avg_words` is how far the transmitter advanced while the handler
ran, in words (one word ≈ 1.25 µs on the wire). With one memory block a half is
24 words, so this is the fraction of a ~30 µs deadline the handler consumes.
Measured with all four channels busy: **4.0–4.9 of 24**, i.e. about a fifth of
the budget, with the interrupt rate at ~133 000 entries/s.

## Memory blocks and how many channels you get

`BLOCKS_PER_CHANNEL = 1` gives every channel a 48-word window, which halves into
24 words = exactly one LED and a refill deadline of ~30 µs — the tightest the
hardware can pose, the honest test of the core's bit-cursor path, and the only
setting that yields all four outputs.

Raising it is a real trade, not a free win: a channel's window extends into the
blocks of the channels *above* it, and those channels stop existing.
`s3_rmt::TX_BLOCKS` is a `lp_ws281x::BlockPlan` built from the constant and
**validated at compile time** — an overlapping plan is a build error. The same
value is handed to the driver and to the backend, so the two cannot disagree
about window sizes, and `Ws281xDriver::configure` rejects an absorbed channel
with `ConfigError::ChannelUnavailable`. See the interrupt-rate table in the
core's README for what each setting costs.

## The loopback self-test (`test_loopback`, phases P3 + P4)

```bash
cd fw/led-lab-esp32s3
cargo run --release --features test_loopback
# or, from the repo root, without holding a monitor open:
espflash reset --port "$XT_PORT_ESP32S3" && python3 scripts/capture.py "$XT_PORT_ESP32S3" 15
```

The feature replaces the demo loop with an on-device timing oracle
(`src/loopback.rs`): **no oscilloscope, no wires, no strips**. Each of GPIO4–7
is split with esp-hal's `Flex::split()` into a frozen input/output signal pair —
the output half drives its RMT TX channel exactly as the demo does, the input
half feeds the paired RMT **RX channel 4–7** through the GPIO matrix (routing
option 1 of the plan; options 2/3 — raw interconnect, physical jumper — were not
needed). The receivers capture every (level, duration) pair at the same 80 MHz /
divider-1 clock, i.e. 12.5 ns resolution, with an idle threshold of 30 000 ticks
(375 µs) so a capture is ended by the post-latch idle and nothing shorter.

All four channels transmit **at once**, under four different configurations, so
every assertion below is made about a channel whose neighbours are competing
with it for the same interrupt handler:

| TX | RX | Timing | Order | LEDs |
|----|----|--------|-------|------|
| 0 | 4 | WS2812 | GRB | 2 |
| 1 | 5 | WS2812 | RGB | 1 |
| 2 | 6 | **WS2811** (300/900 ns) | RGB | 2 |
| 3 | 7 | WS2812 | BGR | 1 |

It prints `E4:` lines and asserts, per channel unless noted:

* `loopback_decode` — every bit classifies by its high time and the decoded
  bytes equal the sent bytes in that channel's byte order (proving `ColorOrder`
  plumbs through configuration, per channel);
* `loopback_timing` — per-bit T0H/T1H and period within ±2 ticks (25 ns) of
  *that channel's* configuration. Measured values are *exactly* nominal (ch0/1/3
  t0h 32, t1h 64; ch2 t0h 24, t1h 72; period 100 ticks everywhere), which is why
  the tolerance is tighter than the spec's ±50 ns ceiling. A channel encoding
  with a neighbour's pulse codes fails here;
* `loopback_latch` — the trailing low bounds the 300 µs latch from below (the
  receiver records the over-threshold idle as a zero-duration marker; the
  threshold itself exceeds the latch);
* `loopback_cross_talk` — no channel's decoded bytes equal any *other*
  channel's expected pattern (and the four patterns are checked to be pairwise
  distinct first, so the check is not vacuous);
* `loopback_soak` — 100 concurrent frames, zero decode mismatches, zero guard
  trips, zero `tx_err`, on all four channels;
* `loopback_truncation` — the dead-man's switch **on silicon, isolated to one
  channel**: `lp-ws281x`'s `test_hooks` feature swallows channel 2's *second*
  threshold interrupt during a 16-LED frame while the other three transmit
  normally. Channel 2 must stop at exactly bit 72 (the guard slot), count
  `guard_trips == 1`, still complete, and leave the line idle rather than
  replaying a stale half; channels 0, 1 and 3 must decode perfectly with
  `guard_trips == 0`, despite sharing every interrupt entry with the victim.

The run ends with `E4: PASS loopback_s3_x4 channels=4 frames=102` (repeated
every 2 s so any capture window catches it), or the first failure's detail.

### RX capacity

Four TX channels take one memory block each, so the four receivers get one block
each too: **48 items**, against P3's 192 for a single receiver. 48 items is
exactly 48 bits — two LEDs — because the S3 records the over-idle-threshold
trailing low as a zero-duration marker inside the final bit's own item rather
than as an extra one. The routine test frames are 1–2 LEDs for that reason.

Longer captures still work, and the truncation test needs one (72 bits): the S3
has RX wrap, so esp-hal drains half a window at a time on `RMT_CH_RX_THR_EVENT`
provided the transaction is polled inside each 24-item (30 µs) window. The
capture loop does nothing else, and the TX interrupt handler is the only thing
that preempts it.

### Re-deriving the golden vector

`lp-ws281x/tests/golden/ws2812_grb_esp32s3.txt` is channel 0's GRB capture,
checked in verbatim (repo rule: golden vectors are hardware-verified, never
hand-written). To re-derive it, run the loopback as above and transcribe the
`E4: MEASURE golden_pairs` lines' `H<ticks>`/`L<ticks>` tokens into the file,
keeping the provenance header accurate (chip, date, config, frame).
`cargo test -p lp-ws281x` then validates the transcription: the vector must
decode to the sent frame and sit within ±25 ns of the configured timing. P4's
four-channel harness reproduces P3's single-channel capture byte for byte.

## The stress harness (`test_stress`, phase P6) — the S3 shrugs off radio load

```bash
cd fw/led-lab-esp32s3
STRESS_SECONDS=600 cargo build --release --features test_stress
```

Same four channels (300/256/200/150 LEDs) as the demo/loopback, run at maximum
frame rate from thread context while an escalating load runs beside them: S0
idle, S1 verbose logging, S2 a WiFi scan loop, S3 ESP-NOW broadcast spam, S4
STA + traffic (skipped without `LED_LAB_WIFI_SSID` at build time — this
firmware carries no TCP/IP stack). The radio stack (`esp-radio`/`esp-rtos`/
`esp-alloc`) is pulled in **only** under this feature, so the demo and
loopback builds stay radio-free.

Authoritative 600 s/scenario result (an earlier, shorter capture on this board
was contaminated by a second process flashing and logging to the same port
concurrently — see `findings.md` §11.1 in the plan directory for the
correction):

| scenario | frames | fps/ch | truncated | lag_max/24 | lag_over_half |
|----------|--------|--------|-----------|------------|----------------|
| S0 idle | 220 568 | 91.9 | **0** | 9 | 0 |
| S1 logging | 39 412 | 16.4 | **0** | 6 | 0 |
| S2 WiFi scan | 216 792 | 90.3 | 2 248 (1.04 %) | 12 | 0 |
| S3 ESP-NOW | 74 692 | 31.1 | 2 | 10 | 0 |

Zero errors, zero timeouts in every scenario. Trips under S2 concentrate
unevenly across channels (ch3/150 LEDs 2 057, ch1 155, ch0 36, ch2 0) rather
than spreading evenly, and `lag_max` never exceeds 12 of the 24-word deadline
even in the worst cell — the refills that ran had comfortable margin, which is
the signature of scarcity (some refills never arrive) rather than lateness
(see the classic ESP32's README for the contrasting case, where `lag_max`
climbs with the trip rate). This is the best result of the three chips by
roughly two orders of magnitude; `guard_trips` is a **lower bound** here (the
S3 re-raises an unacknowledged `tx_thr_event` — see above), so the true rate
is at most 1.04 %, not more.

Two bench notes recorded so a future run is not misread:

* **This board's WiFi receiver is effectively dead** (`aps=0` throughout,
  even idle) — a 13-channel passive sweep found one beacon at −96 dBm (noise
  floor) while a C6 beside it sees 3–7 APs through the identical driver and
  config. That is a property of this board (~40 dB down, consistent with an
  unconnected antenna), not of the firmware, and it does not weaken S2: the
  scan still runs and costs CPU whether or not anything answers — 3 871 scans
  (6.4/s), 0 scan errors. Do not use this board to measure anything about
  *received* WiFi.
* **A real `esp-radio` hang was found and worked around.** `scan_async`
  awaits `ScanDone` on a 2-message pub-sub channel whose `next_message_pure`
  silently skips an evicted message (`WaitResult::Lagged(_) => continue`), so
  an evicted `ScanDone` leaves the future waiting forever for an event that is
  never republished. Mitigated with a 5 s per-scan deadline that drops the
  future, counts it (`E6: MEASURE … scan_deadlines=`), and lets the hardware
  scan drain in the background. Fired 0 times in the 600 s run above; worth
  reporting upstream to `esp-radio`.

## Build and run (demo)

```bash
cd fw/led-lab-esp32s3
cargo build --release
cargo run --release            # flash the S3 and open a monitor
```

From the repo root, to capture without holding a monitor open:

```bash
python3 scripts/capture.py "$XT_PORT_ESP32S3" 10
```

The crate declares its own `[workspace]`. `fw/` is excluded from the repo's root
workspace (different toolchain, different target), but an *ancestor* of the
checkout can still claim it — a git worktree under `.claude/worktrees/` does
exactly that — so the root is pinned here.

## Design notes

* **No `static mut`, no `steal`, no `transmute` to `'static`.** The driver is a
  plain `static Ws281xDriver<S3Rmt, 4>`: `Ws281xDriver::new` is `const` and
  every field of `ChannelState` is an atomic, so thread context and the handler
  share a `&'static` and nothing needs unsafe. The esp-hal `Channel` is held in
  `main` for the life of the program, which is what keeps the pin and the memory
  reservation alive — the registry problem that forced `AnyPin::steal` in lp2025
  does not exist in a lab firmware that owns its pins.
* **The ISR is a trampoline** (`DRIVER.on_interrupt()`) placed in IRAM with
  `#[esp_hal::ram]`; a flash-cache miss in the refill path is exactly the
  latency the guard word exists to survive, so it should not be self-inflicted.
  The `lp-ws281x` body it calls still lives in flash — moving that into IRAM too
  is a question for the stress phase, not this one.
* **The handler is bound once**, behind an `AtomicBool`, closing the
  re-registration `TODO` the lp2025 driver carried.
* **Frames are sent with the safe API** wherever the shape allows it.
  `send_blocking_all` starts every channel back to back and borrows all four
  frames for the whole transmission, aborting each one on any exit path; the
  spin closure aborts after 50 ms so a missing interrupt produces a `FAIL` line
  instead of a silent hang. Free-running mode is the one place that uses the
  `unsafe` `start_frame`, because no single call spans the transmission — the
  frame buffers live for the whole program and are only rewritten once
  `is_complete(ch)`.

## Provenance

Original code. The esp-hal call sequence (channel acquisition, `conf_update`
dance, interrupt registration) follows the author's own ESP32-C6 driver in
`lp2025` (`lp-fw/fw-esp32/src/output/rmt/`); register and field names were
derived from the `esp32s3` PAC and esp-hal's MIT/Apache-2.0 RMT driver. **No GPL
source was consulted** — in particular not WLED. See `AGENTS.md` and
[`docs/adr/2026-07-28-license-provenance-discipline.md`](../../docs/adr/2026-07-28-license-provenance-discipline.md).
