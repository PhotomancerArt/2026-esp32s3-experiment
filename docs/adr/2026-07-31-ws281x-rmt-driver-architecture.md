# ADR: Multi-channel WS281x driver — per-channel RMT ping-pong + guard word

- Status: accepted
- Date: 2026-07-31
- Deciders: Yona Appletree
- Relates to: plan `2026-07-28-ws281x-rmt-driver` (P1–P7), `lp-ws281x` (the
  portable core), `fw/led-lab-esp32s3`/`fw/led-lab-esp32`/`fw/led-lab-esp32c6`
  (the three chip backends), `findings.md` in the plan directory (the P6
  stress data this ADR's HLI section cites), and the lp2025 backport's own
  plan (`~/.photomancer/planning/lp2025/2026-07-31-0720-s3-led-output-4ch/`),
  which ports `lp-ws281x` unmodified — see `docs/BACKPORT.md`.

## Context

lp2025 has a single-channel WS281x driver for the ESP32-C6
(`lp-fw/fw-esp32/src/output/rmt/`), written by the same author against that
chip's RMT peripheral. This project needed the same idea generalized to
N channels and ported to two more chips — the classic ESP32 (the actual
deployment target: commercial 4-output "WLED controller" boards run this
chip, not the C6) and the ESP32-S3 (the desk bring-up board) — while fixing
a list of known warts: a start-of-frame guard race, channel 0 hardcoded in
the ISR, `static mut` + `transmute('static)` + `AnyPin::steal`, per-`new`
interrupt re-registration, a `Relaxed`-store/`Acquire`-load counter mismatch,
hardcoded GRB, and a 50 µs latch too short for WS2812B-V5/WS2815.

WS281x is clockless: every bit is a fixed-period pulse whose high time
carries the value, to roughly ±150 ns tolerance. The RMT peripheral transmits
such pulses from a RAM window far too small to hold a frame (48–64 words
against 24 words per LED), so any driver is a refill race — keep the half
the transmitter just left full of fresh pulses, forever, from an interrupt
handler that competes with WiFi. The central engineering question is how
much of that race margin a given approach leaves, and what happens when it
runs out.

## Decision

**Per-channel RMT ping-pong refill, driven by a bit-cursor core over an
`RmtHw` backend trait, with a guard-word dead-man's switch and per-channel
timing/color-order configuration.**

- **RMT, not DMA.** Chosen over parallel-DMA driving (I2S1 on the classic
  ESP32, LCD_CAM on the S3) because it gives genuinely independent
  per-channel timing and color order — a parallel-DMA lane count is fixed at
  configure time and every lane shares one clock — and because RMT is the
  peripheral lp2025's existing driver already used, so the port is an
  evolution, not a rewrite. See "Alternatives" for why DMA is deferred, not
  rejected.
- **A bit cursor, not an LED counter.** lp2025 counted LEDs and assumed a
  half held a whole number of them (a 192-word window on the C6, halving to
  96 words = exactly 4 LEDs). That assumption breaks the moment a channel
  owns one RMT memory block instead of four: a 48-word block (S3, C6) halves
  into 24 words = exactly one LED, and the classic ESP32's 64-word block
  halves into 32 words = 1⅓ LEDs. Tracking bit position instead of LED
  position makes every half size work on every chip, and turns
  `blocks_per_channel` into a free tuning knob (see the ISR-ceiling section
  below) instead of a hardcoded assumption.
- **The `RmtHw` trait is the entire chip-specific surface** (`lp-ws281x/src/hw.rs`):
  `ram_words`, `write_ram`, `set_tx_threshold`, `read_pos`, `start_tx`,
  `stop_tx`, `take_interrupts` — seven operations, all `&self`, none of them
  deciding *what* to write or *when*. Every sequencing decision (which half
  to refill, where the guard goes, when the threshold flips, when a frame is
  done) lives in `Ws281xDriver` and is tested once on the host against a mock
  transmitter, then reused unchanged across three RMT generations with
  different channel counts (8/4/2), block sizes (64/48/48 words) and register
  layouts. No chip backend contains a sequencing decision; a bug found on one
  chip is fixed once, in the core, and every backend inherits the fix.
- **The guard word is a dead-man's switch, not a heartbeat.** After each
  refill the driver plants an all-zero STOP word at the first word of the
  half the transmitter is currently reading — a slot already consumed, and
  the one it would next re-read if the following refill interrupt never
  arrives. A lost interrupt therefore truncates the frame instead of
  replaying a stale half forever: one dim frame instead of visible flicker.
  Two behaviors differ deliberately from lp2025, both fixing the same class
  of race: nothing is planted at start (lp2025 planted at word 0 right after
  `tx_start` and hoped "with any luck" the transmitter had already passed
  it — a real, lost race), and the guard slot is checked against the read
  pointer before planting, so an implausibly fast handler cannot kill a
  healthy frame (counted as `guard_skips` instead, which is routine — not a
  bug — when several channels share one interrupt line).
- **Per-channel `ChannelTiming` + `ColorOrder`**, defaulting to WS2812 at
  800 kHz with a 300 µs latch (WLED/NeoPixelBus's number for modern parts;
  lp2025's 50 µs is too short for WS2812B-V5 and WS2815). Configurable
  per-channel is what an RMT-per-channel design buys over parallel DMA, where
  every lane shares one clock and one latch.

## Chip support matrix

| Chip | ISA | RMT TX channels | RAM/channel | RAM offset | Role |
|---|---|---|---|---|---|
| Classic ESP32 | Xtensa LX6 | 8, each TX-or-RX | 64 words | `+0x800` | **deployment target** — commercial 4-output WLED-class controllers |
| ESP32-S3 | Xtensa LX7 | 4, fixed TX | 48 words | `+0x800` | desk bring-up board, should-support |
| ESP32-C6 | RISC-V | 2, fixed TX | 48 words | `+0x400` | lp2025's current board; origin chip of the ancestor driver |

The three RAM offsets and three `INT_*` bit layouts are three *distinct*
register shapes behind **identical PAC accessor names** — a silent-porting
trap documented per-backend and repeated in `docs/BACKPORT.md` for whoever
ports a fourth chip. All three chips nonetheless produce the identical wire
waveform for the same configuration — asserted cross-chip in
`lp-ws281x/tests/golden_esp32c6.rs` and `hardware_golden.rs`, not just
claimed — which is the evidence that the `RmtHw` abstraction boundary is
drawn in the right place.

## `blocks_per_channel` and the ISR-ceiling trade

A channel can be given more than one RMT memory block, extending its window
into the blocks immediately above it (`BlockPlan`, validated so an
overlapping plan is a compile-time or configure-time error, never silent
RAM corruption). More blocks means a longer refill deadline and a lower
interrupt rate, at the cost of fewer channels — `BlockPlan` makes that trade
explicit instead of implicit.

P6's stress phase found this trade is not just about latency margin; on the
classic ESP32 it is a hard **throughput ceiling**. The delivered interrupt
rate flatlines at ~46,000–55,000/s regardless of how much refill work is
demanded (`findings.md` §12, the sweep's own `irq_hz` column). A
continuously-transmitting channel demands `800,000 / half_words` refills/s —
25,000/s at the shipped 32-word half — so two channels demand exactly
50,000/s, which is where the ceiling sits: below it every channel gets
everything it asks for, at or above it the losing channels' frames truncate
on (deterministically) every frame. That arithmetic is why the classic ships
today at `blocks_per_channel = 1` (all eight channels available, ceiling
reached at two *simultaneous equal-length* outputs) rather than the
`blocks_per_channel = 2` a 4-output product wants (~64-word halves, ~4
channels of headroom against the same ceiling — implemented behind an env
override, not yet validated on silicon; see `findings.md` §12 "What was NOT
completed"). Coincident deadlines (several channels started together with
identical lengths) are a real secondary effect on top of the ceiling, not an
independent cause — a stagger experiment improved but did not eliminate the
failure (`findings.md` §12).

On the ESP32-S3 and ESP32-C6 the same knob is a latency-margin question, not
yet a throughput one: `blocks_per_channel = 1` gives four/two channels a
24-word half (~30 µs deadline) with 4.0–4.9 words of measured margin at full
fan-out (`lp-ws281x/README.md`), comfortably inside the deadline in every P6
scenario except the classic's radio-load cases below.

## Alternatives considered

### esp-hal-smartled — rejected as the driver, kept as a day-1 smoke reference

esp-hal's own smart-LED helper buffers one RMT item per bit in RAM rather
than refilling a ping-pong window: ~96 bytes/LED, which puts a 3,200-LED
install (a real 4×800-LED deployment target) at 300 KB — it does not fit on
a classic ESP32's SRAM alongside WiFi. Its own documentation flags it as
interrupt-latency-sensitive with no flicker-protection mechanism comparable
to the guard word. Useful only as an initial smoke test that the toolchain
and pin routing work, never as the shipping driver.

### Parallel DMA (I2S1 on the classic ESP32, LCD_CAM on the S3) — deferred, not rejected

An 8-lane parallel-clockless DMA driver (the WLED/NeoPixelBus and
`I2SClockless*` prior-art shape) needs roughly 72 bytes/LED of DMA buffer
independent of lane count, and is immune to interrupt latency entirely — no
refill race exists once the descriptor chain is armed. It was not chosen for
this phase because: no Rust implementation of it exists to build from (every
prior-art reference is C/C++); every lane shares one clock and one latch
duration, which loses the per-channel timing/color-order independence RMT
gives for free; and RMT was sufficient for the classic ESP32's and S3's
channel counts (8 and 4) once the guard word and the throughput-ceiling
arithmetic above were understood.

**Revisit triggers** (from the plan; none has fired as of P7): guard trips
under ordinary load that a higher-priority RMT interrupt cannot fix, more
than 4 simultaneous outputs needed on the ESP32-S3 (a hard ceiling — it has
only 4 TX channels, so more outputs are only reachable via DMA or a
different chip), or RAM pressure from long strips that the RMT RAM budget
cannot absorb even at `blocks_per_channel = 1`.

### High-priority interrupt (HLI) shim — was measurement-gated, now **GO**

WLED's `NeoESP32RmtHI` demonstrates a level-4/5 Xtensa interrupt shim that
preempts the WiFi stack's own interrupt priority, derived from Espressif's
`hli_vector.S`. This project deliberately did not build one speculatively:
P6 exists to measure whether normal-priority RMT interrupts actually lose
the race under realistic radio load, and only recommend the shim if they do.

They do, decisively, on the deployment target. From `findings.md` (§8.1,
§11.1, §11.4, the authoritative post-addendum figures):

| board | WiFi scan truncation | ESP-NOW truncation | worst refill margin under scan |
|---|---|---|---|
| **classic ESP32** | **66–69 %** | **50–51 %** | **2–2.5 µs of a 40 µs deadline** |
| ESP32-C6 | 29 % | 0.08 % | ~11 µs of a 30 µs deadline |
| ESP32-S3 | 1.04 % | ~0.003 % | ~16 µs of a 30 µs deadline |

The classic result is qualitatively different from the other two boards: the
refill lag *climbed with the trip rate* under WiFi scan (11 words idle → 30
of 32 under load, the only genuine deadline **overrun** measured anywhere in
this project), which is the direct signature of an interrupt being held off
by higher-priority radio work rather than a structural or throughput-ceiling
issue (those show low lag *and* high trips, the opposite signature — see the
`blocks_per_channel` section above and `findings.md` §6 for how to tell the
two apart). ESP-NOW is lp2025's actual radio usage, not an edge case, and it
costs the classic ESP32 one frame in two. **Decision: build the Xtensa HLI
follow-up as its own plan**, clean-room from Espressif's Apache-licensed
`hli_vector.S` — WLED's GPL shim is off-limits as a source and unnecessary as
one, since its behavior is already captured in this project's own notes. The
ESP32-C6's 29 % is a real defect too but needs no assembly: RISC-V has no
priority-vector shim to write, only the RMT interrupt's priority to raise
above the radio's — smaller, and worth doing first (`findings.md` §8.2). The
classic ESP32 also has an independent, unrelated defect (the ISR-throughput
ceiling above) that the HLI shim will **not** fix by itself, since it is a
saturation problem, not a latency one — a faster, less-preemptible ISR does
raise the ceiling, but that is a capacity argument, not the interrupt-racing
argument this section is about.

## License posture

- **WLED and NeoPixelBus (`NeoESP32RmtHI`) are GPL** (WLED GPL-3.0, NeoPixelBus
  LGPL-3.0's copyleft implications treated the same way here) — **behavioral
  reference only**: read to understand what a parallel-DMA driver or an HLI
  shim needs to do, never copied or transliterated. Their timing table (WS2811
  300/950+900/350 ns, WS2812x 400/850+800/450 ns, 300 µs latch) informed this
  driver's default `ChannelTiming::WS2812`/`WS2811` constants as *facts about
  the protocol* (the datasheets independently state the same numbers), not as
  copied expression.
- **esp-hal (MIT/Apache-2.0), the per-chip PACs, and `esp-metadata-generated`**
  are the license-safe sources for every register fact in the three backends —
  RAM offsets, `INT_*` layouts, `CH_TX_LIM`/`mem_raddr_ex` semantics — each
  cited at the point of use in the backend source and the crate READMEs, per
  `docs/adr/2026-07-28-license-provenance-discipline.md`'s established
  discipline for this repo.
- **Espressif's `hli_vector.S` (Apache-2.0)** is recorded here as the intended
  clean-room source for the HLI follow-up plan's level-4/5 vector, should that
  plan proceed — not built in this phase, but the license posture is decided
  now so the follow-up plan does not have to re-litigate it.
- **The `lp-ws281x` core and all three backends are original code**, evolved
  from the author's own lp2025 driver (no license issue — the author's own
  prior work) with a deliberate cleanup rather than a straight port. No GPL
  source was consulted for any of it.

## Consequences

- A bug found in the sequencing logic (guard placement, ping-pong flip, bit
  accounting) is fixed once in `lp-ws281x` and every chip backend inherits the
  fix without a register-level change — demonstrated across P1–P6, where zero
  sequencing bugs were chip-specific and every chip-specific bug (the
  classic's `CH_TX_LIM` repeating-count semantics, its RX-arms-corrupt-TX
  RMT-RAM-fetch defect, the S3's `tx_thr_event` re-raise) was isolated to one
  backend file.
- `guard_trips` is an exact count on the classic ESP32 and the C6, but only a
  **lower bound** on the ESP32-S3 (it re-raises an unacknowledged
  `tx_thr_event`, so a truncation in a frame's final half can go uncounted).
  Any telemetry or product decision consuming this counter across chips must
  carry that asymmetry — it is documented in `lp-ws281x/README.md` and
  `ChannelStats::guard_trips`'s doc comment, not just here.
- The classic ESP32 — the actual deployment target — cannot yet drive more
  than two simultaneous *equal-length* outputs cleanly at the shipped
  `blocks_per_channel = 1`, and needs the (implemented, not yet
  silicon-validated) `blocks_per_channel = 2` configuration to reach four.
  This is a real, currently-open product constraint, not merely a
  measurement footnote — see `findings.md` §12 for the full arithmetic and
  what remains to validate it.
- Two follow-up plans are implied by this ADR's GO decision and are explicitly
  **not** started here: the Xtensa HLI shim (§ above), and validating
  `blocks_per_channel = 2` on the classic ESP32 (the sweep harness's
  configure-time plumbing for absorbed blocks is unfinished — `findings.md`
  §12). Both are out of scope for the cleanup phase this ADR was written in.
