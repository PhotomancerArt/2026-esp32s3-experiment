//! `led-lab-esp32c6` — the ESP32-C6 backend for `lp-ws281x`.
//!
//! Drives **both** ESP32-C6 RMT TX channels concurrently, each with its own
//! strip length, wire timing and byte order, and prints machine-checkable
//! status, so the driver can be regression-tested with nothing attached to the
//! board:
//!
//! ```text
//! E1: MEASURE rmt_base=0x60006000 rmt_ram=0x60006400 ram_offset=0x400 ...
//! E1: PASS rmt_ram_offset direct=1 fifo=1 base=1
//! E2: MEASURE ch=0 leds=8 frames=30 guard_trips=0 guard_skips=0 errors=0 \
//!     refill_lag_avg_words=3.6 timeouts=0
//! E2: PASS ws281x_c6_basic channels=2 frames_advancing=1 mode=simultaneous
//! ```
//!
//! Strips on the data pins are a bonus visual check; the pass/fail signal does
//! not depend on them.
//!
//! # Pins (Seeed XIAO ESP32-C6)
//!
//! | Signal | GPIO | XIAO label | Notes |
//! |--------|------|------------|-------|
//! | Channel 0 data | 18 | **D10** | RMT `CH0` via the GPIO matrix; lp2025's WS281x output header |
//! | Channel 1 data | 20 | **D9** | RMT `CH1` |
//! | Debug | 4 | *(none)* | High for the duration of channel 0's frame; three fast pulses when a guard trips |
//!
//! GPIO18/GPIO20 are confirmed as D10/D9 against lp2025's own board manifest
//! (`lp-core/lpc-hardware/boards/seeed/xiao-esp32-c6.json`, which records GPIO18
//! as the "known WS281x output header"). GPIO4 is **not broken out** on the XIAO
//! — the same manifest gives it no board label — so the debug marker is only
//! probeable at the module, not at a castellation. It is unreserved there (the
//! manifest flags GPIO12/13 as unsafe, not GPIO4) and is the C6's MTMS pin,
//! which is idle while the board is debugged over USB-Serial-JTAG. D3/GPIO21 is
//! the nearest header-accessible alternative if a scope probe is ever needed.
//!
//! # Start modes
//!
//! The demo alternates between the two ways the outputs get used, because they
//! stress different things:
//!
//! * **simultaneous** — both frames start within a few register writes of each
//!   other, so the channels cross their half boundaries in lockstep and the
//!   handler is entered with two coincident thresholds;
//! * **free-running** — each channel has its own frame interval and restarts as
//!   soon as it is idle, so the refill requests arrive at unrelated phases and
//!   `tx_end` for one channel routinely lands in the same snapshot as a refill
//!   for the other.
//!
//! # Division of labour
//!
//! * [`lp_ws281x::Ws281xDriver`] — all sequencing, tested on the host.
//! * [`c6_rmt::C6Rmt`] — the seven register operations, and the only place in
//!   this firmware that knows an ESP32-C6 address.
//! * `main` — esp-hal shell (clock, GPIO matrix, interrupt binding), the
//!   patterns, and the serial protocol.

#![no_std]
#![no_main]

#[cfg(all(feature = "test_loopback", feature = "test_stress"))]
compile_error!(
    "test_loopback and test_stress are mutually exclusive build modes: the loopback \
     harness owns the RX channels and asserts a quiet wire, the stress harness \
     transmits on every channel while a radio runs beside it"
);

mod c6_rmt;
#[cfg(feature = "test_loopback")]
mod loopback;
#[cfg(feature = "test_stress")]
mod stress;

use core::sync::atomic::{AtomicBool, Ordering};

use esp_hal::clock::CpuClock;
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::main;
use esp_hal::rmt::Rmt;
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
use esp_hal::rmt::{TxChannelConfig, TxChannelCreator};
use esp_hal::time::{Duration, Instant, Rate};

use lp_ws281x::Ws281xDriver;
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
use lp_ws281x::{ChannelTiming, ColorOrder};

use c6_rmt::{C6Rmt, TX_BLOCKS, TX_CHANNELS};
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
use c6_rmt::{BLOCKS_PER_CHANNEL, CHANNEL_WORDS, RAM_BASE, RAM_OFFSET};

esp_bootloader_esp_idf::esp_app_desc!();

/// RMT source clock. Divider 1 makes one tick 12.5 ns, which is what
/// [`lp_ws281x::PulseCodes::DEFAULT_CLOCK_HZ`] assumes.
///
/// On the C6 this is `PLL_F80M`, not the APB clock: esp-hal picks the source
/// that can produce the requested rate, and the C6's APB runs at 40 MHz once
/// `CpuClock::max()` (160 MHz) is applied. `Rmt::new` fails rather than silently
/// halving the tick rate, which the `E1: FAIL rmt_init` path reports.
const RMT_CLOCK: Rate = Rate::from_mhz(80);

/// The reference channel: the RAM probe's subject and the debug pin's frame
/// marker. The other one is its equal in every other respect. (The loopback
/// harness addresses both by index and has no use for it.)
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const CH: u8 = 0;

/// LEDs per channel — deliberately unequal, so the two frames end at different
/// times and the handler never settles into a single rhythm.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const STRIP_LEDS: [usize; TX_CHANNELS] = [8, 100];

/// The longest frame, in bytes: sizes the per-channel buffers.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const MAX_FRAME_BYTES: usize = 100 * 3;

/// Per-channel frame interval in free-running mode — mutually non-harmonic, so
/// the phase relationship between the channels keeps drifting.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const FREE_INTERVAL_US: [u64; TX_CHANNELS] = [17_000, 29_000];

/// Frame interval in simultaneous mode (~30 fps for the pair).
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const FRAME_INTERVAL: Duration = Duration::from_micros(33_333);

/// How long one start mode runs before the demo switches to the other.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const MODE_INTERVAL: Duration = Duration::from_millis(5000);

/// A frame that has not completed within this long has hung; abort it and say
/// so rather than spinning forever. The longest frame (100 LEDs) is ~3 ms.
const FRAME_TIMEOUT: Duration = Duration::from_millis(50);

/// How often the `MEASURE`/`PASS` block is printed.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const REPORT_INTERVAL: Duration = Duration::from_millis(1000);

/// Sentinels for the RMT RAM address probe. Neither is a legal pulse word, and
/// neither is zero (which would be a STOP marker and thus indistinguishable
/// from freshly cleared RAM).
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const DIRECT_SENTINEL: u32 = 0xA5A5_5A5A;
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
const FIFO_SENTINEL: u32 = 0x1234_ABCD;

/// The driver, shared between `main` and the interrupt handler.
///
/// `Ws281xDriver::with_blocks` is `const`, and every field of `ChannelState` is
/// an atomic, so this needs neither `static mut` nor a `StaticCell` — the
/// handler and thread context simply share a `&'static`.
static DRIVER: Ws281xDriver<C6Rmt, TX_CHANNELS> =
    Ws281xDriver::with_blocks(C6Rmt::new(TX_BLOCKS), TX_BLOCKS);

/// Set once the RMT interrupt handler has been bound.
///
/// lp2025 re-registered the handler on every channel construction with a
/// `TODO` about it; this is that TODO.
static ISR_INSTALLED: AtomicBool = AtomicBool::new(false);

/// The RMT interrupt entry point: a trampoline and nothing else.
///
/// Placed in IRAM with `#[ram]` — a flash-cache miss here is exactly the
/// latency the guard word exists to survive, so it should not be self-inflicted.
/// (The driver body it calls still lives in flash; moving `lp-ws281x` into IRAM
/// as well is a stress-phase question, not this phase's.)
///
/// One entry can service both channels: with a block each they cross their half
/// boundaries within microseconds of one another, so coincident causes are the
/// rule rather than the exception. That single pass is
/// [`Ws281xDriver::on_interrupt`]'s job, not this trampoline's.
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

    #[cfg(feature = "test_stress")]
    stress::run(peripherals);

    #[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
    demo(peripherals)
}

/// Per-channel wire configuration: two different strips on one peripheral.
///
/// Channel 0 keeps the plain WS2812/GRB setup the golden vectors were captured
/// with; channel 1 exists to prove the configuration is genuinely per channel
/// and not a global the handler happens to read.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
fn channel_timings() -> [ChannelTiming; TX_CHANNELS] {
    [
        ChannelTiming::WS2812,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Rgb),
    ]
}

/// Which start mode the demo is currently in.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Both channels started together, then waited for together.
    Simultaneous,
    /// Each channel restarts on its own interval, independently.
    FreeRunning,
}

#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
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

/// The demo: two independent chases on GPIO18/GPIO20 plus the `E1`/`E2` serial
/// protocol.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
fn demo(peripherals: esp_hal::peripherals::Peripherals) -> ! {
    esp_println::println!(
        "led-lab-esp32c6: ws281x RMT driver, {} channels on GPIO18/GPIO20 ({} LEDs), \
         debug on GPIO4",
        TX_CHANNELS,
        STRIP_LEDS[0],
    );

    // Frame-boundary marker for a logic analyser: high while channel 0's frame
    // is on the wire, plus a burst when a guard word truncates one.
    let mut debug = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());

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

    // Kept alive for the lifetime of the program: dropping either of these would
    // release that channel's memory block and disconnect its pin. Owning them
    // properly is why this firmware needs no `AnyPin::steal` or `transmute` to
    // `'static`.
    let _channels = match (
        rmt.channel0.configure_tx(&config),
        rmt.channel1.configure_tx(&config),
    ) {
        (Ok(c0), Ok(c1)) => [
            c0.with_pin(peripherals.GPIO18),
            c1.with_pin(peripherals.GPIO20),
        ],
        _ => {
            esp_println::println!("E1: FAIL rmt_configure reason=configure_tx");
            halt();
        }
    };

    // --- E1: is the RMT RAM where we think it is? ---
    let probe = c6_rmt::probe_ram_address(&TX_BLOCKS, CH, DIRECT_SENTINEL, FIFO_SENTINEL);
    esp_println::println!(
        "E1: MEASURE rmt_base={:#010x} rmt_ram={:#010x} ram_offset={:#x} \
         pac_base={:#010x} channel_words={} blocks_per_channel={} tx_channels={} \
         available_channels={}",
        RAM_BASE - RAM_OFFSET,
        RAM_BASE,
        RAM_OFFSET,
        probe.peripheral_base,
        CHANNEL_WORDS,
        BLOCKS_PER_CHANNEL,
        TX_CHANNELS,
        TX_BLOCKS.available_channels(),
    );
    let direct_ok = probe.direct_readback == DIRECT_SENTINEL;
    let fifo_ok = probe.fifo_readback == FIFO_SENTINEL;
    let base_ok = probe.peripheral_base == c6_rmt::RMT_BASE;
    if probe.ok(DIRECT_SENTINEL, FIFO_SENTINEL) {
        esp_println::println!(
            "E1: PASS rmt_ram_offset direct={} fifo={} base={}",
            direct_ok as u8,
            fifo_ok as u8,
            base_ok as u8,
        );
    } else {
        esp_println::println!(
            "E1: FAIL rmt_ram_offset direct={} fifo={} base={} direct_readback={:#010x} \
             fifo_readback={:#010x}",
            direct_ok as u8,
            fifo_ok as u8,
            base_ok as u8,
            probe.direct_readback,
            probe.fifo_readback,
        );
    }

    c6_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

    let timings = channel_timings();
    for (ch, timing) in timings.iter().enumerate() {
        if let Err(e) = DRIVER.configure_default_clock(ch as u8, timing) {
            esp_println::println!("E2: FAIL ws281x_c6_basic reason=configure_ch{ch}:{e:?}");
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

                let starts: [(u8, &[u8]); TX_CHANNELS] = [
                    (0, &storage[0][..STRIP_LEDS[0] * 3]),
                    (1, &storage[1][..STRIP_LEDS[1] * 3]),
                ];

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
                    esp_println::println!("E2: FAIL ws281x_c6_basic reason=start:{e:?}");
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
                        esp_println::println!("E2: FAIL ws281x_c6_basic reason=start:{e:?}");
                    }
                    sent_at[ch] = Instant::now();
                    due[ch] = sent_at[ch] + Duration::from_micros(FREE_INTERVAL_US[ch]);
                }

                // Nothing here waits on a single channel: the sweep above is the
                // whole loop body, so a long frame on one channel never delays a
                // short one on the other.
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
                    "E2: FAIL ws281x_c6_basic reason=frames_stalled mode={}",
                    mode.name()
                );
            } else if trips != 0 {
                // Idle here means: no WiFi, nothing else running. A guard trip
                // in these conditions is a driver bug, not a load symptom.
                esp_println::println!(
                    "E2: FAIL ws281x_c6_basic reason=idle_guard_trip guard_trips_delta={trips} \
                     mode={}",
                    mode.name()
                );
            } else if errs != 0 {
                esp_println::println!(
                    "E2: FAIL ws281x_c6_basic reason=tx_err errors_delta={errs} mode={}",
                    mode.name()
                );
            } else if hangs != 0 {
                esp_println::println!(
                    "E2: FAIL ws281x_c6_basic reason=frame_timeout timeouts_delta={hangs} mode={}",
                    mode.name()
                );
            } else {
                esp_println::println!(
                    "E2: PASS ws281x_c6_basic channels={} frames_advancing=1 mode={}",
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
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
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
/// so two strips side by side are told apart at a glance.
#[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
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
#[cfg_attr(any(feature = "test_loopback", feature = "test_stress"), allow(dead_code))]
fn halt() -> ! {
    loop {
        let park = Instant::now();
        while park.elapsed() < Duration::from_millis(1000) {}
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    #[cfg(feature = "test_loopback")]
    esp_println::println!("E5: FAIL loopback_esp32c6 reason=panic info={info}");
    #[cfg(feature = "test_stress")]
    esp_println::println!("E6: FAIL stress_c6 reason=panic info={info}");
    #[cfg(not(any(feature = "test_loopback", feature = "test_stress")))]
    esp_println::println!("E2: FAIL ws281x_c6_basic reason=panic info={info}");
    esp_hal::system::software_reset()
}
