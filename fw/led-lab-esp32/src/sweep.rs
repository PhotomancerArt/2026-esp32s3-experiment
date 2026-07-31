//! P6 — the channel-count sweep: how many outputs can the classic ESP32 drive?
//!
//! P5's 8-TX soak left an open finding: with **no** receivers armed and nothing
//! else running, the longer strips took guard trips (ch5: 439 trips in 903
//! frames) while the four-channel demo never took one. That looks like plain
//! interrupt starvation — eight channels each want a refill every 32 wire bits
//! (~40 µs), so eight of them together ask the handler for ~200 000 refills a
//! second — and if it is, it bounds how many outputs this chip can ship with.
//!
//! This build walks the matrix and prints one machine-readable line per cell so
//! the bound can be read off rather than guessed at:
//!
//! * channel counts 1..8, every active channel the **same** length so their
//!   half-boundary crossings coincide (the worst phase relation, not an average
//!   one);
//! * two strip lengths, [`SWEEP_LEDS`] — a short frame spends proportionally
//!   more time in the latch, where nothing refills, so length changes the duty
//!   cycle of the load as well as its length;
//! * every cell run for [`CELL_SECS`] seconds at the maximum frame rate the
//!   wire allows.
//!
//! # Why TX-only, and why no receiver
//!
//! P5 root-caused a hardware interaction on this chip: arming **two or more**
//! RMT RX channels while another channel transmits corrupts the transmitter's
//! pad output — its RAM fetch returns a receiver's write. So there is no way to
//! witness the wire here without changing the thing being measured. The sweep
//! is therefore TX + counters only, and the counters are the whole signal:
//! `guard_trips` says a frame was truncated, `refill_lag_max` says how close
//! the worst refill came to the deadline, and `lag_hist`'s last bucket says how
//! often there was no margin left at all.
//!
//! # What the numbers mean — and the trap in them
//!
//! The deadline is one ping-pong **half**: 32 words on the classic ESP32 (a
//! 64-word block per channel).
//!
//! `refill_lag` is how far the transmitter's read pointer advanced **while a
//! refill was running** — the handler's own cost, expressed in wire words
//! (1 word = 1.25 µs at 800 kHz). It is *not* how late the refill was: the
//! clock starts when this channel's refill starts, so every microsecond spent
//! waiting for the interrupt, for the ISR prologue, or for another channel's
//! refill earlier in the same pass is invisible to it.
//!
//! That matters because it makes the obvious reading of a starved run wrong.
//! A refill that never arrives at all leaves **no lag sample behind**, so a
//! channel can truncate every single frame while its `lag_max` sits at a third
//! of the deadline. `lag_max` climbing toward 32 would mean the refills got
//! *slower*; it is not the signal that they got *scarcer*. For that, compare
//! `refills` with `refills_wanted` on each line — a frame of `leds` LEDs needs
//! `leds * 24 / 32` refills, and getting fewer is starvation however healthy
//! the lag looks.
//!
//! ```text
//! E7: MEASURE cell=8x300 ch=5 leds=300 frames=903 complete=464 trips=439 ...
//! E7: CELL channels=8 leds=300 secs=30 frames=7224 trips=1732 lag_max=32 \
//!     half=32 lag_over_half=8801 refills=5940000 irq_hz=198000
//! ```

use esp_hal::gpio::Level;
use esp_hal::peripherals::Peripherals;
use esp_hal::rmt::{Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::{Duration, Instant};

use lp_ws281x::{ChannelTiming, LAG_BUCKETS};

use crate::esp32_rmt::{self, BLOCKS_PER_CHANNEL, TX_BLOCKS, TX_CHANNELS};
use crate::{install_isr, DRIVER, RMT_CLOCK};

/// Strip lengths swept, in LEDs, for every channel count.
///
/// 100 and 300: a 100-LED frame is 2400 bits (~3.0 ms) followed by the same
/// 300 µs latch as a 300-LED frame (~9.0 ms), so the short cell asks for the
/// same *instantaneous* refill rate at a ~91 % duty cycle instead of ~97 %.
pub const SWEEP_LEDS: [usize; 2] = [100, 300];

/// The longest strip in [`SWEEP_LEDS`] — sizes the static frame buffers.
const MAX_LEDS: usize = 300;

/// Bytes in the longest frame.
const MAX_FRAME_BYTES: usize = MAX_LEDS * 3;

/// Seconds per cell. Sixteen cells, so the default is a ~8 minute run;
/// `SWEEP_SECONDS=5 cargo build …` shortens it for a smoke test.
/// RMT ticks to offset each channel's start from the previous one. `0` keeps
/// the original coincident-boundary worst case. A bit is 100 ticks and a
/// 32-word half is 3200 ticks, so e.g. 1067 spreads three channels evenly
/// across one half.
///
/// NOTE: `option_env!` is read at compile time and does **not** make cargo
/// rebuild when the variable changes — `touch` this file after changing it or
/// you will flash a stale binary.
const STAGGER_TICKS: u32 = match option_env!("SWEEP_STAGGER_TICKS") {
    Some(s) => match u32::from_str_radix(s, 10) {
        Ok(v) => v,
        Err(_) => 0,
    },
    None => 0,
};

/// Spacing between usable channel slots. A channel given `n` blocks absorbs the
/// next `n-1` channels' blocks, so with 2 blocks each only slots 0, 2, 4, 6 can
/// be configured. Cell index `i` therefore drives channel `i * SLOT_STRIDE`.
const SLOT_STRIDE: usize = BLOCKS_PER_CHANNEL as usize;

/// Usable channels under the current block plan.
const USABLE_CHANNELS: usize = TX_CHANNELS / SLOT_STRIDE;

const CELL_SECS: u64 = match option_env!("SWEEP_SECONDS") {
    Some(s) => parse_u64(s),
    None => 30,
};

/// Decimal `u64` parser for [`CELL_SECS`]; a malformed value is a build
/// failure, not a silently wrong sweep length.
const fn parse_u64(s: &str) -> u64 {
    let bytes = s.as_bytes();
    let mut i = 0;
    let mut acc = 0u64;
    while i < bytes.len() {
        let d = bytes[i];
        assert!(d.is_ascii_digit(), "SWEEP_SECONDS must be a decimal integer");
        acc = acc * 10 + (d - b'0') as u64;
        i += 1;
    }
    assert!(acc > 0, "SWEEP_SECONDS must be positive");
    acc
}

/// The frames transmitted, compiled into flash and never modified.
///
/// Static and immutable is what makes handing their addresses to the interrupt
/// handler sound with no `unsafe` beyond `start_frame`'s own contract. The
/// content is a deterministic gradient rather than zeros — a zero and a one bit
/// take the same 1.25 µs on the wire, so the pattern does not change the
/// timing, but a varied one is readable on a scope and visible on a strip.
static FRAMES: [[u8; MAX_FRAME_BYTES]; TX_CHANNELS] = build_frames();

const fn build_frames() -> [[u8; MAX_FRAME_BYTES]; TX_CHANNELS] {
    let mut frames = [[0u8; MAX_FRAME_BYTES]; TX_CHANNELS];
    let mut ch = 0;
    while ch < TX_CHANNELS {
        let mut i = 0;
        while i < MAX_FRAME_BYTES {
            frames[ch][i] = ((i * 7 + ch * 53) % 256) as u8;
            i += 1;
        }
        ch += 1;
    }
    frames
}

pub fn run(peripherals: Peripherals) -> ! {
    esp_println::println!(
        "led-lab-esp32: P6 channel-count sweep, up to {} TX channels, leds={:?}, {} s/cell",
        TX_CHANNELS,
        SWEEP_LEDS,
        CELL_SECS,
    );

    let mut rmt = match Rmt::new(peripherals.RMT, RMT_CLOCK) {
        Ok(rmt) => rmt,
        Err(e) => {
            esp_println::println!("E7: FAIL sweep_esp32 reason=rmt_init:{e:?}");
            crate::halt();
        }
    };
    install_isr(&mut rmt);

    let config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output(true)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_memsize(BLOCKS_PER_CHANNEL);

    // Every channel is configured up front, whatever the cell uses. Dropping
    // one would release its memory block and disconnect its pin — and dropping
    // the last would gate the RMT clock — so they are kept alive for the whole
    // sweep and the block plan never changes between cells. That is deliberate:
    // a channel count is the only variable, not the memory layout.
    let _channels = match (
        rmt.channel0.configure_tx(&config),
        rmt.channel1.configure_tx(&config),
        rmt.channel2.configure_tx(&config),
        rmt.channel3.configure_tx(&config),
        rmt.channel4.configure_tx(&config),
        rmt.channel5.configure_tx(&config),
        rmt.channel6.configure_tx(&config),
        rmt.channel7.configure_tx(&config),
    ) {
        (Ok(c0), Ok(c1), Ok(c2), Ok(c3), Ok(c4), Ok(c5), Ok(c6), Ok(c7)) => (
            c0.with_pin(peripherals.GPIO16),
            c1.with_pin(peripherals.GPIO17),
            c2.with_pin(peripherals.GPIO18),
            c3.with_pin(peripherals.GPIO19),
            c4.with_pin(peripherals.GPIO22),
            c5.with_pin(peripherals.GPIO23),
            c6.with_pin(peripherals.GPIO25),
            c7.with_pin(peripherals.GPIO26),
        ),
        _ => {
            esp_println::println!("E7: FAIL sweep_esp32 reason=configure_tx");
            crate::halt();
        }
    };

    esp32_rmt::init_tx();
    esp32_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

    // Identical wire timing on every channel, unlike the demo's deliberately
    // mixed set: the sweep compares channel *counts*, so the per-channel bit
    // rate has to be a constant. (WS2811's 300/950 + 900/350 ns is the same
    // 1.25 µs period as WS2812's, but there is no reason to introduce the
    // variable at all.)
    for ch in 0..TX_CHANNELS {
        if let Err(e) = DRIVER.configure_default_clock(ch as u8, &ChannelTiming::WS2812) {
            esp_println::println!("E7: FAIL sweep_esp32 reason=configure_ch{ch}:{e:?}");
            crate::halt();
        }
    }

    let half = DRIVER.channel(0).map(|c| c.half_words()).unwrap_or(0);
    esp_println::println!(
        "E7: START sweep_esp32 counts=1..{} leds={:?} secs_per_cell={} half_words={} \
         blocks_per_channel={}",
        TX_CHANNELS,
        SWEEP_LEDS,
        CELL_SECS,
        half,
        BLOCKS_PER_CHANNEL,
    );

    for &leds in SWEEP_LEDS.iter() {
        for count in 1..=USABLE_CHANNELS {
            run_cell(count, leds);
        }
    }

    esp_println::println!("E7: DONE sweep_esp32 cells={}", SWEEP_LEDS.len() * TX_CHANNELS);
    crate::halt()
}

/// One cell of the matrix: `count` channels, `leds` LEDs on each, for
/// [`CELL_SECS`] seconds at the maximum frame rate.
fn run_cell(count: usize, leds: usize) {
    for ch in 0..TX_CHANNELS {
        if let Some(state) = DRIVER.channel(ch as u8) {
            state.reset_stats();
        }
    }

    let bytes = leds * 3;
    // Built for all eight and sliced: `send_blocking_all` takes a slice, so the
    // active set is `[..count]` and the rest are never armed.
    let starts: [(u8, &[u8]); TX_CHANNELS] = core::array::from_fn(|i| {
        ((i * SLOT_STRIDE) as u8, &FRAMES[i][..bytes])
    });
    let active = &starts[..count];

    esp_println::println!(
        "E7: CELL_START channels={count} leds={leds} secs={CELL_SECS} stagger_ticks={STAGGER_TICKS}"
    );

    let started = Instant::now();
    let limit = Duration::from_millis(CELL_SECS * 1000);
    let mut timeouts = 0usize;
    let mut start_errors = 0usize;

    while started.elapsed() < limit {
        let round = Instant::now();
        let mut timed_out = false;
        let spin = || {
            if round.elapsed() > crate::FRAME_TIMEOUT {
                timed_out = true;
                for i in 0..count {
                    DRIVER.abort((i * SLOT_STRIDE) as u8);
                }
            }
        };
        // With `STAGGER_TICKS` = 0 (the default) all `count` channels are
        // started back to back — a few register writes apart, i.e. simultaneous
        // as far as a 1.25 µs bit is concerned. Equal lengths plus a common
        // start is the coincident-threshold worst case the cell is for.
        //
        // Non-zero spaces the starts out instead, to test whether that
        // coincidence is what breaks channels 3..n: see `send_staggered`.
        let send = if STAGGER_TICKS == 0 {
            DRIVER.send_blocking_all(active, spin)
        } else {
            send_staggered(active, STAGGER_TICKS, spin)
        };
        if timed_out {
            timeouts += 1;
        }
        if send.is_err() {
            start_errors += 1;
        }
    }

    let secs = started.elapsed().as_secs().max(1);
    report_cell(count, leds, secs, timeouts, start_errors);
}

/// CPU cycles per RMT tick: 240 MHz core, 12.5 ns tick (80 MHz source).
const CYCLES_PER_TICK: u32 = 3;

/// Cycle counter. `esp_hal` re-exports `xtensa_lx`, so this needs no inline asm
/// (still unstable on this architecture) and no `unsafe`.
#[inline(always)]
fn ccount() -> u32 {
    esp_hal::xtensa_lx::timer::get_cycle_count()
}

/// Busy-wait `ticks` RMT ticks. Wrap-safe: the unsigned difference stays in the
/// top half of the range only while `until` is still ahead.
fn spin_ticks(ticks: u32) {
    let until = ccount().wrapping_add(ticks * CYCLES_PER_TICK);
    while ccount().wrapping_sub(until) > u32::MAX / 2 {}
}

/// Start each channel `stagger` RMT ticks after the previous one, then wait for
/// all of them — the same shape as [`lp_ws281x::Ws281xDriver::send_blocking_all`]
/// but with the starts deliberately spread out.
///
/// The point is the *threshold* boundaries, not the starts: channels running
/// equal-length frames from a common start cross every half boundary together,
/// so all `n` threshold interrupts land at once and the handler must service
/// them in sequence. Offsetting the starts offsets those boundaries by the same
/// amount for the whole frame.
fn send_staggered(
    active: &[(u8, &[u8])],
    stagger: u32,
    mut spin: impl FnMut(),
) -> Result<(), lp_ws281x::StartError> {
    for (i, (ch, frame)) in active.iter().enumerate() {
        // SAFETY: `active` borrows the frames for this whole call, and every
        // channel is driven to completion or aborted before it returns, so the
        // bytes the handler reads through its raw pointer stay alive, in place
        // and unmodified for the entire transmission.
        if let Err(e) = unsafe { DRIVER.start_frame(*ch, frame) } {
            for (c, _) in active {
                DRIVER.abort(*c);
            }
            return Err(e);
        }
        if stagger > 0 && i + 1 < active.len() {
            spin_ticks(stagger);
        }
    }

    while active.iter().any(|(ch, _)| !DRIVER.is_complete(*ch)) {
        spin();
    }
    Ok(())
}

/// One `MEASURE` line per active channel, then the cell's `CELL` summary.
fn report_cell(count: usize, leds: usize, secs: u64, timeouts: usize, start_errors: usize) {
    let mut frames = 0usize;
    let mut trips = 0usize;
    let mut skips = 0usize;
    let mut errors = 0usize;
    let mut lag_max = 0i32;
    let mut over_half = 0u32;
    let mut refills = 0u64;
    let mut refills_wanted = 0u64;
    let mut hist = [0u32; LAG_BUCKETS];

    for i in 0..count {
        let ch = i * SLOT_STRIDE;
        let stats = DRIVER.stats(ch as u8);
        let half = DRIVER
            .channel(ch as u8)
            .map(|c| c.half_words())
            .unwrap_or(0);
        // Refills an untruncated frame of this length needs: one per half
        // boundary the frame crosses. Comparing it with the refills that
        // actually happened is the whole starvation signal — a channel that
        // fell short did not get its interrupts, whatever the lag figures say.
        let wanted_per_frame = if half == 0 {
            0
        } else {
            (leds * 24).div_ceil(half)
        };
        let (lag_int, lag_frac) = crate::mean_lag_tenths(stats.refill_lag_sum, stats.refill_lag_count);
        esp_println::println!(
            "E7: MEASURE cell={}x{} ch={} leds={} secs={} frames={} complete={} \
             trips={} skips={} errors={} lag_avg={}.{} lag_max={} half={} \
             refills={} refills_wanted={} lag_over_half={} lag_hist={}",
            count,
            leds,
            ch,
            leds,
            secs,
            stats.frames,
            stats.complete_frames(),
            stats.guard_trips,
            stats.guard_skips,
            stats.errors,
            lag_int,
            lag_frac,
            stats.refill_lag_max,
            half,
            stats.refill_lag_count,
            stats.frames as u64 * wanted_per_frame as u64,
            stats.lag_over_half(),
            HistFmt(stats.lag_hist),
        );

        frames += stats.frames;
        trips += stats.guard_trips;
        skips += stats.guard_skips;
        errors += stats.errors;
        over_half += stats.lag_over_half();
        refills += stats.refill_lag_count.max(0) as u64;
        refills_wanted += stats.frames as u64 * wanted_per_frame as u64;
        if stats.refill_lag_max > lag_max {
            lag_max = stats.refill_lag_max;
        }
        for (slot, v) in hist.iter_mut().zip(stats.lag_hist.iter()) {
            *slot += v;
        }
    }

    let half = DRIVER.channel(0).map(|c| c.half_words()).unwrap_or(0);
    // The x-axis of the starvation story, in two parts. `irq_hz_demand` is what
    // the wire asked for: every frame that was started needed one refill per
    // half boundary it crosses, whether or not it got them. `irq_hz` is what
    // the handler actually delivered. They track each other while there is
    // headroom and diverge the moment the CPU runs out — which is a far
    // sharper signal than the lag figures, because a refill that never arrives
    // leaves no lag sample behind at all.
    let irq_hz = refills / secs;
    let irq_hz_demand = refills_wanted / secs;
    // Frames per second across the whole set, in tenths.
    let fps_tenths = (frames as u64 * 10) / secs;
    esp_println::println!(
        "E7: CELL channels={} leds={} secs={} frames={} complete={} fps={}.{} \
         trips={} skips={} errors={} timeouts={} start_errors={} lag_max={} half={} \
         lag_over_half={} refills={} refills_wanted={} irq_hz={} irq_hz_demand={} \
         lag_hist={}",
        count,
        leds,
        secs,
        frames,
        frames - trips.min(frames),
        fps_tenths / 10,
        fps_tenths % 10,
        trips,
        skips,
        errors,
        timeouts,
        start_errors,
        lag_max,
        half,
        over_half,
        refills,
        refills_wanted,
        irq_hz,
        irq_hz_demand,
        HistFmt(hist),
    );
}

/// Comma-separated histogram, without dragging formatter machinery in.
struct HistFmt([u32; LAG_BUCKETS]);

impl core::fmt::Display for HistFmt {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (i, v) in self.0.iter().enumerate() {
            if i > 0 {
                write!(f, ",")?;
            }
            write!(f, "{v}")?;
        }
        Ok(())
    }
}
