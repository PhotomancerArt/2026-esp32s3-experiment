//! `led-lab-esp32` — the classic-ESP32 (LX6) backend for `lp-ws281x`.
//!
//! Drives four RMT TX channels concurrently (all **eight** under the
//! `soak_8tx` feature), each with its own strip length, wire timing and byte
//! order, and prints machine-checkable status over UART0, so the driver can be
//! regression-tested with nothing attached to the board:
//!
//! ```text
//! E1: MEASURE rmt_base=0x3ff56000 rmt_ram=0x3ff56800 ram_offset=0x800 ...
//! E1: PASS rmt_ram_offset direct=1 fifo=1
//! E2: MEASURE ch=0 leds=8 frames=30 guard_trips=0 guard_skips=0 errors=0 \
//!     refill_lag_avg_words=3.6 timeouts=0
//! E2: PASS ws281x_esp32_basic channels=4 frames_advancing=1 mode=simultaneous
//! ```
//!
//! Strips on the data pins are a bonus visual check; the pass/fail signal does
//! not depend on them.
//!
//! # Pins
//!
//! GPIO 6–11 (flash), 0/2/12/15 (strapping) and 34–39 (input-only) are
//! avoided.
//!
//! | Signal | GPIO | Notes |
//! |--------|------|-------|
//! | Channel 0 data | 16 | RMT `CH0` via the GPIO matrix |
//! | Channel 1 data | 17 | RMT `CH1` |
//! | Channel 2 data | 18 | RMT `CH2` |
//! | Channel 3 data | 19 | RMT `CH3` |
//! | Channel 4 data | 22 | `soak_8tx` only |
//! | Channel 5 data | 23 | `soak_8tx` only |
//! | Channel 6 data | 25 | `soak_8tx` only |
//! | Channel 7 data | 26 | `soak_8tx` only |
//! | Debug | 21 | High for the duration of channel 0's frame; three fast pulses when a guard trips |
//!
//! # Start modes
//!
//! The demo alternates between the two ways multiple outputs get used, because
//! they stress different things:
//!
//! * **simultaneous** — all frames start within a few register writes of each
//!   other, so every channel crosses its half boundaries in lockstep and the
//!   handler is entered with coincident thresholds (up to eight under
//!   `soak_8tx`);
//! * **free-running** — each channel has its own frame interval and restarts
//!   as soon as it is idle, so the refill requests arrive at unrelated phases
//!   and `tx_end` for one channel routinely lands in the same snapshot as a
//!   refill for another.
//!
//! # Division of labour
//!
//! * [`lp_ws281x::Ws281xDriver`] — all sequencing, tested on the host.
//! * [`esp32_rmt::Esp32Rmt`] — the seven register operations, and the only
//!   place in this firmware that knows a classic-ESP32 address.
//! * `main` — esp-hal shell (clock, GPIO matrix, interrupt binding), the
//!   patterns, and the serial protocol.

#![no_std]
#![no_main]

#[cfg(all(feature = "test_loopback", feature = "soak_8tx"))]
compile_error!(
    "test_loopback and soak_8tx are mutually exclusive build modes: the loopback \
     harness needs channels 4-7 as receivers, the soak transmits on them"
);
#[cfg(all(feature = "diag", any(feature = "test_loopback", feature = "soak_8tx")))]
compile_error!(
    "diag is its own build mode: it owns channels 0-7 (four TX + four RX) and \
     replaces the main loop, exactly as test_loopback does"
);
#[cfg(all(
    feature = "sweep_channels",
    any(feature = "test_loopback", feature = "diag", feature = "soak_8tx")
))]
compile_error!(
    "sweep_channels is its own build mode: it owns all eight channels as \
     transmitters and replaces the main loop"
);
#[cfg(all(
    feature = "test_stress",
    any(
        feature = "test_loopback",
        feature = "diag",
        feature = "soak_8tx",
        feature = "sweep_channels"
    )
))]
compile_error!(
    "test_stress is its own build mode: it owns four channels plus the radio \
     stack and replaces the main loop"
);

#[cfg(feature = "diag")]
mod diag;
mod esp32_rmt;
#[cfg(feature = "test_loopback")]
mod loopback;
#[cfg(feature = "test_stress")]
mod stress;
#[cfg(feature = "sweep_channels")]
mod sweep;

use core::sync::atomic::{AtomicBool, Ordering};

use esp_hal::clock::CpuClock;
#[cfg(demo_build)]
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::main;
use esp_hal::rmt::Rmt;
#[cfg(demo_build)]
use esp_hal::rmt::{TxChannelConfig, TxChannelCreator};
use esp_hal::time::{Duration, Instant, Rate};

use lp_ws281x::Ws281xDriver;
#[cfg(demo_build)]
use lp_ws281x::{ChannelTiming, ColorOrder};

use esp32_rmt::{Esp32Rmt, TX_BLOCKS, TX_CHANNELS};
#[cfg(demo_build)]
use esp32_rmt::{BLOCKS_PER_CHANNEL, CHANNEL_WORDS, RAM_BASE, RAM_OFFSET};

esp_bootloader_esp_idf::esp_app_desc!();

/// RMT source clock: the 80 MHz APB clock (`ref_always_on` = 1, which esp-hal
/// sets for every channel when handed this rate). Divider 1 makes one tick
/// 12.5 ns, which is what [`lp_ws281x::PulseCodes::DEFAULT_CLOCK_HZ`] assumes.
/// The classic ESP32 has no fractional RMT prescaler, so `Rmt::new` accepts
/// exactly this frequency and nothing else.
const RMT_CLOCK: Rate = Rate::from_mhz(80);

/// What the `E2:` lines call this experiment.
#[cfg(demo_build)]
const TEST_NAME: &str = if cfg!(feature = "soak_8tx") {
    "ws281x_esp32_soak8"
} else {
    "ws281x_esp32_basic"
};

/// The reference channel: the RAM probe's subject and the debug pin's frame
/// marker. The others are its equals in every other respect. (The loopback
/// harness addresses channels by index and has no use for it.)
#[cfg(demo_build)]
const CH: u8 = 0;

/// LEDs per channel — deliberately unequal, so the frames end at different
/// times and the handler never settles into a single rhythm.
#[cfg(all(demo_build, not(feature = "soak_8tx")))]
const STRIP_LEDS: [usize; TX_CHANNELS] = [8, 16, 100, 256];
#[cfg(all(demo_build, feature = "soak_8tx"))]
const STRIP_LEDS: [usize; TX_CHANNELS] = [8, 16, 100, 256, 32, 64, 150, 200];

/// The longest frame, in bytes: sizes the per-channel buffers.
#[cfg(demo_build)]
const MAX_FRAME_BYTES: usize = 256 * 3;

/// Per-channel frame interval in free-running mode — mutually non-harmonic, so
/// the phase relationship between the channels keeps drifting.
#[cfg(all(demo_build, not(feature = "soak_8tx")))]
const FREE_INTERVAL_US: [u64; TX_CHANNELS] = [17_000, 23_000, 29_000, 33_000];
#[cfg(all(demo_build, feature = "soak_8tx"))]
const FREE_INTERVAL_US: [u64; TX_CHANNELS] = [
    17_000, 23_000, 29_000, 33_000, 19_000, 27_000, 31_000, 37_000,
];

/// Frame interval in simultaneous mode (~30 fps for the whole set).
#[cfg(demo_build)]
const FRAME_INTERVAL: Duration = Duration::from_micros(33_333);

/// How long one start mode runs before the demo switches to the other.
#[cfg(demo_build)]
const MODE_INTERVAL: Duration = Duration::from_millis(5000);

/// A frame that has not completed within this long has hung; abort it and say
/// so rather than spinning forever. The longest frame (256 LEDs) is ~7.7 ms.
const FRAME_TIMEOUT: Duration = Duration::from_millis(50);

/// How often the `MEASURE`/`PASS` block is printed.
#[cfg(demo_build)]
const REPORT_INTERVAL: Duration = Duration::from_millis(1000);

/// Sentinels for the RMT RAM address probe. Neither is a legal pulse word, and
/// neither is zero (which would be a STOP marker and thus indistinguishable
/// from freshly cleared RAM).
#[cfg(demo_build)]
const DIRECT_SENTINEL: u32 = 0xA5A5_5A5A;
#[cfg(demo_build)]
const FIFO_SENTINEL: u32 = 0x1234_ABCD;

/// The driver, shared between `main` and the interrupt handler.
///
/// `Ws281xDriver::with_blocks` is `const`, and every field of `ChannelState`
/// is an atomic, so this needs neither `static mut` nor a `StaticCell` — the
/// handler and thread context simply share a `&'static`.
static DRIVER: Ws281xDriver<Esp32Rmt, TX_CHANNELS> =
    Ws281xDriver::with_blocks(Esp32Rmt::new(TX_BLOCKS), TX_BLOCKS);

/// Set once the RMT interrupt handler has been bound.
static ISR_INSTALLED: AtomicBool = AtomicBool::new(false);

/// The RMT interrupt entry point: a trampoline and nothing else.
///
/// Placed in IRAM with `#[ram]` — a flash-cache miss here is exactly the
/// latency the guard word exists to survive, so it should not be
/// self-inflicted. One entry can service every channel: with a block each they
/// cross their half boundaries within microseconds of one another, so
/// coincident causes are the rule rather than the exception.
#[esp_hal::ram]
extern "C" fn rmt_isr() {
    DRIVER.on_interrupt();
}

/// Bind [`rmt_isr`] at the highest priority esp-hal can dispatch, exactly once.
fn install_isr(rmt: &mut Rmt<'_, esp_hal::Blocking>) {
    if ISR_INSTALLED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return;
    }
    rmt.set_interrupt_handler(InterruptHandler::new(rmt_isr, Priority::max()));
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    #[cfg(feature = "test_loopback")]
    loopback::run(peripherals);

    #[cfg(feature = "diag")]
    diag::run(peripherals);

    #[cfg(feature = "sweep_channels")]
    sweep::run(peripherals);

    #[cfg(feature = "test_stress")]
    stress::run(peripherals);

    #[cfg(demo_build)]
    demo(peripherals)
}

/// Per-channel wire configuration: different strips on one peripheral.
///
/// Channel 0 keeps the plain WS2812/GRB setup the golden vectors were captured
/// with; the rest exist to prove the configuration is genuinely per channel
/// and not a global the handler happens to read. The `soak_8tx` build repeats
/// the cycle over the second bank of four.
#[cfg(demo_build)]
fn channel_timings() -> [ChannelTiming; TX_CHANNELS] {
    let cycle = [
        ChannelTiming::WS2812,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Rgb),
        ChannelTiming::WS2811,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Bgr),
    ];
    core::array::from_fn(|ch| cycle[ch % 4])
}

/// Which start mode the demo is currently in.
#[cfg(demo_build)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// All channels started together, then waited for together.
    Simultaneous,
    /// Each channel restarts on its own interval, independently.
    FreeRunning,
}

#[cfg(demo_build)]
impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Simultaneous => "simultaneous",
            Mode::FreeRunning => "free_running",
        }
    }

    fn flip(self) -> Self {
        match self {
            Mode::Simultaneous => Mode::FreeRunning,
            Mode::FreeRunning => Mode::Simultaneous,
        }
    }
}

/// The demo: independent chases on the data pins plus the `E1`/`E2` serial
/// protocol. Doubles as the 8-TX soak under `soak_8tx` (same loop, all eight
/// channels, no receivers — the guard-trip/error counters are the signal).
#[cfg(demo_build)]
fn demo(peripherals: esp_hal::peripherals::Peripherals) -> ! {
    esp_println::println!(
        "led-lab-esp32: ws281x RMT driver, {} channels ({} LEDs on ch0), debug on GPIO21",
        TX_CHANNELS,
        STRIP_LEDS[0],
    );

    // Frame-boundary marker for a logic analyser: high while channel 0's frame
    // is on the wire, plus a burst when a guard word truncates one.
    let mut debug = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());

    let mut rmt = match Rmt::new(peripherals.RMT, RMT_CLOCK) {
        Ok(rmt) => rmt,
        Err(e) => {
            esp_println::println!("E1: FAIL rmt_init reason={e:?}");
            halt();
        }
    };
    install_isr(&mut rmt);

    let config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output(true)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_memsize(BLOCKS_PER_CHANNEL);

    // Kept alive for the lifetime of the program: dropping any of these would
    // release that channel's memory block and disconnect its pin.
    #[cfg(not(feature = "soak_8tx"))]
    let _channels = match (
        rmt.channel0.configure_tx(&config),
        rmt.channel1.configure_tx(&config),
        rmt.channel2.configure_tx(&config),
        rmt.channel3.configure_tx(&config),
    ) {
        (Ok(c0), Ok(c1), Ok(c2), Ok(c3)) => [
            c0.with_pin(peripherals.GPIO16),
            c1.with_pin(peripherals.GPIO17),
            c2.with_pin(peripherals.GPIO18),
            c3.with_pin(peripherals.GPIO19),
        ],
        _ => {
            esp_println::println!("E1: FAIL rmt_configure reason=configure_tx");
            halt();
        }
    };
    #[cfg(feature = "soak_8tx")]
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
            esp_println::println!("E1: FAIL rmt_configure reason=configure_tx");
            halt();
        }
    };

    esp32_rmt::init_tx();

    // --- E1: is the RMT RAM where we think it is? ---
    let probe = esp32_rmt::probe_ram_address(&TX_BLOCKS, CH, DIRECT_SENTINEL, FIFO_SENTINEL);
    esp_println::println!(
        "E1: MEASURE rmt_base={:#010x} rmt_ram={:#010x} ram_offset={:#x} \
         channel_words={} blocks_per_channel={} tx_channels={} available_channels={}",
        RAM_BASE - RAM_OFFSET,
        RAM_BASE,
        RAM_OFFSET,
        CHANNEL_WORDS,
        BLOCKS_PER_CHANNEL,
        TX_CHANNELS,
        TX_BLOCKS.available_channels(),
    );
    let direct_ok = probe.direct_readback == DIRECT_SENTINEL;
    let fifo_ok = probe.fifo_readback == FIFO_SENTINEL;
    if probe.ok(DIRECT_SENTINEL, FIFO_SENTINEL) {
        esp_println::println!(
            "E1: PASS rmt_ram_offset direct={} fifo={}",
            direct_ok as u8,
            fifo_ok as u8
        );
    } else {
        esp_println::println!(
            "E1: FAIL rmt_ram_offset direct={} fifo={} direct_readback={:#010x} \
             fifo_readback={:#010x}",
            direct_ok as u8,
            fifo_ok as u8,
            probe.direct_readback,
            probe.fifo_readback,
        );
    }

    esp32_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

    let timings = channel_timings();
    for (ch, timing) in timings.iter().enumerate() {
        if let Err(e) = DRIVER.configure_default_clock(ch as u8, timing) {
            esp_println::println!("E2: FAIL {TEST_NAME} reason=configure_ch{ch}:{e:?}");
            halt();
        }
    }

    // One buffer per channel, sized for the longest strip. They live for the
    // rest of the program — `demo` never returns — which is what makes handing
    // their addresses to the interrupt handler sound.
    let mut storage = [[0u8; MAX_FRAME_BYTES]; TX_CHANNELS];
    let mut heads = [0usize; TX_CHANNELS];

    let mut timeouts = [0usize; TX_CHANNELS];
    let mut last_frames = [0usize; TX_CHANNELS];
    let mut last_guard_trips = [0usize; TX_CHANNELS];
    let mut last_errors = [0usize; TX_CHANNELS];
    let mut last_timeouts = [0usize; TX_CHANNELS];
    // Tracked per frame rather than per report, so the debug burst fires once
    // per trip instead of on every frame until the next report.
    let mut pulsed_guard_trips = 0usize;

    let mut mode = Mode::Simultaneous;
    let mut mode_started = Instant::now();
    let mut last_report = Instant::now();
    // Free-running mode's per-channel next-start deadline, and when the frame
    // currently in flight was started (the hang detector's reference).
    let mut due = [Instant::now(); TX_CHANNELS];
    let mut sent_at = [Instant::now(); TX_CHANNELS];

    loop {
        match mode {
            Mode::Simultaneous => {
                let round_started = Instant::now();
                for ch in 0..TX_CHANNELS {
                    render_chase(&mut storage[ch][..STRIP_LEDS[ch] * 3], heads[ch], ch);
                    heads[ch] = (heads[ch] + 1) % STRIP_LEDS[ch];
                }

                let starts: [(u8, &[u8]); TX_CHANNELS] =
                    core::array::from_fn(|ch| (ch as u8, &storage[ch][..STRIP_LEDS[ch] * 3]));

                debug.set_high();
                let mut timed_out = false;
                let send = DRIVER.send_blocking_all(&starts, || {
                    if round_started.elapsed() > FRAME_TIMEOUT {
                        timed_out = true;
                        for ch in 0..TX_CHANNELS {
                            DRIVER.abort(ch as u8);
                        }
                    }
                });
                debug.set_low();

                if timed_out {
                    for t in timeouts.iter_mut() {
                        *t += 1;
                    }
                }
                if let Err(e) = send {
                    esp_println::println!("E2: FAIL {TEST_NAME} reason=start:{e:?}");
                }

                while round_started.elapsed() < FRAME_INTERVAL {}
            }
            Mode::FreeRunning => {
                for ch in 0..TX_CHANNELS {
                    if !DRIVER.is_complete(ch as u8) {
                        // Still on the wire; a frame started elsewhere in this
                        // sweep must not be disturbed.
                        continue;
                    }
                    if Instant::now() < due[ch] {
                        continue;
                    }
                    render_chase(&mut storage[ch][..STRIP_LEDS[ch] * 3], heads[ch], ch);
                    heads[ch] = (heads[ch] + 1) % STRIP_LEDS[ch];

                    if ch == CH as usize {
                        debug.set_high();
                    }
                    // SAFETY: `storage` lives until the program ends (`demo`
                    // diverges), so the bytes stay in place for the whole
                    // transmission. They are only rewritten above, which this
                    // loop reaches solely when `is_complete(ch)` — i.e. after
                    // the handler has stopped reading them. A frame that
                    // overruns `FRAME_TIMEOUT` is aborted below, which also
                    // clears the handler's pointer.
                    let started =
                        unsafe { DRIVER.start_frame(ch as u8, &storage[ch][..STRIP_LEDS[ch] * 3]) };
                    if let Err(e) = started {
                        esp_println::println!("E2: FAIL {TEST_NAME} reason=start:{e:?}");
                    }
                    sent_at[ch] = Instant::now();
                    due[ch] = sent_at[ch] + Duration::from_micros(FREE_INTERVAL_US[ch]);
                }

                // Nothing here waits on a single channel: the sweep above is
                // the whole loop body, so a long frame on one channel never
                // delays a short one on another.
                for ch in 0..TX_CHANNELS {
                    if !DRIVER.is_complete(ch as u8) && sent_at[ch].elapsed() > FRAME_TIMEOUT {
                        DRIVER.abort(ch as u8);
                        timeouts[ch] += 1;
                    }
                }
                if DRIVER.is_complete(CH) {
                    debug.set_low();
                }
            }
        }

        let total_trips: usize = (0..TX_CHANNELS)
            .map(|ch| DRIVER.stats(ch as u8).guard_trips)
            .sum();
        if total_trips > pulsed_guard_trips {
            pulsed_guard_trips = total_trips;
            // Three fast pulses: visually distinct from the frame marker in a
            // capture, and cheap enough to do outside the handler.
            for _ in 0..3 {
                debug.set_high();
                debug.set_low();
            }
        }

        if last_report.elapsed() >= REPORT_INTERVAL {
            let mut advancing = true;
            let mut trips = 0usize;
            let mut errs = 0usize;
            let mut hangs = 0usize;

            for ch in 0..TX_CHANNELS {
                let stats = DRIVER.stats(ch as u8);
                let (lag_int, lag_frac) =
                    mean_lag_tenths(stats.refill_lag_sum, stats.refill_lag_count);
                esp_println::println!(
                    "E2: MEASURE ch={} leds={} frames={} guard_trips={} guard_skips={} \
                     errors={} refill_lag_avg_words={}.{} timeouts={}",
                    ch,
                    STRIP_LEDS[ch],
                    stats.frames,
                    stats.guard_trips,
                    stats.guard_skips,
                    stats.errors,
                    lag_int,
                    lag_frac,
                    timeouts[ch],
                );

                advancing &= stats.frames > last_frames[ch];
                trips += stats.guard_trips - last_guard_trips[ch];
                errs += stats.errors - last_errors[ch];
                hangs += timeouts[ch] - last_timeouts[ch];

                last_frames[ch] = stats.frames;
                last_guard_trips[ch] = stats.guard_trips;
                last_errors[ch] = stats.errors;
                last_timeouts[ch] = timeouts[ch];
            }

            if !advancing {
                esp_println::println!(
                    "E2: FAIL {TEST_NAME} reason=frames_stalled mode={}",
                    mode.name()
                );
            } else if trips != 0 {
                // Idle here means: no WiFi, nothing else running. A guard trip
                // in these conditions is a driver bug, not a load symptom.
                esp_println::println!(
                    "E2: FAIL {TEST_NAME} reason=idle_guard_trip guard_trips_delta={trips} \
                     mode={}",
                    mode.name()
                );
            } else if errs != 0 {
                esp_println::println!(
                    "E2: FAIL {TEST_NAME} reason=tx_err errors_delta={errs} mode={}",
                    mode.name()
                );
            } else if hangs != 0 {
                esp_println::println!(
                    "E2: FAIL {TEST_NAME} reason=frame_timeout timeouts_delta={hangs} mode={}",
                    mode.name()
                );
            } else {
                esp_println::println!(
                    "E2: PASS {TEST_NAME} channels={} frames_advancing=1 mode={}",
                    TX_CHANNELS,
                    mode.name(),
                );
            }

            last_report = Instant::now();
        }

        if mode_started.elapsed() >= MODE_INTERVAL {
            // Let every channel finish before changing the rhythm, so a mode
            // boundary never looks like a stall.
            let drain = Instant::now();
            while (0..TX_CHANNELS).any(|ch| !DRIVER.is_complete(ch as u8)) {
                if drain.elapsed() > FRAME_TIMEOUT {
                    for ch in 0..TX_CHANNELS {
                        DRIVER.abort(ch as u8);
                    }
                    break;
                }
            }
            mode = mode.flip();
            mode_started = Instant::now();
            due = [Instant::now(); TX_CHANNELS];
        }
    }
}

/// Mean refill lag in words, as an integer part and one decimal digit.
///
/// Done in integers so the report does not drag core's float formatter into a
/// firmware that has no other use for it.
///
/// Shared by every build mode that prints a lag figure (demo, sweep, stress);
/// the loopback and diag harnesses report raw counts instead.
#[cfg_attr(any(feature = "test_loopback", feature = "diag"), allow(dead_code))]
fn mean_lag_tenths(sum: i32, count: i32) -> (i32, i32) {
    if count == 0 {
        return (0, 0);
    }
    let tenths = sum.saturating_mul(10) / count;
    (tenths / 10, (tenths % 10).abs())
}

/// A four-pixel comet with a decaying tail, one pixel further along each frame.
///
/// The colour changes every lap so a stalled pattern is obvious to the eye as
/// well as to the `MEASURE` line, and the starting colour differs per channel
/// so strips side by side are told apart at a glance.
#[cfg(demo_build)]
fn render_chase(frame: &mut [u8], head: usize, ch: usize) {
    frame.fill(0);
    let leds = frame.len() / 3;
    if leds == 0 {
        return;
    }

    let lap = (head / 4 + ch) % 3;
    for tail in 0..4usize {
        let led = (head + leds * 4 - tail) % leds;
        // 255, 63, 15, 3 — a visible falloff that stays non-zero.
        let level = (255u32 >> (tail * 2)) as u8;
        let base = led * 3;
        frame[base + lap] = level;
    }
}

/// Stop, having already said why.
#[cfg_attr(not(demo_build), allow(dead_code))]
fn halt() -> ! {
    loop {
        let park = Instant::now();
        while park.elapsed() < Duration::from_millis(1000) {}
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    #[cfg(feature = "test_loopback")]
    esp_println::println!("E5: FAIL loopback_esp32 reason=panic info={info}");
    #[cfg(feature = "diag")]
    esp_println::println!("D5: MEASURE fatal reason=panic info={info}");
    #[cfg(feature = "sweep_channels")]
    esp_println::println!("E7: FAIL sweep_esp32 reason=panic info={info}");
    #[cfg(feature = "test_stress")]
    esp_println::println!("E6: FAIL stress_esp32 reason=panic info={info}");
    #[cfg(demo_build)]
    esp_println::println!("E2: FAIL {TEST_NAME} reason=panic info={info}");
    esp_hal::system::software_reset()
}
