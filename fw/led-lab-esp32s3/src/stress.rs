//! P6 — the stress harness: does the refill interrupt survive radio load?
//!
//! Four channels of long strips transmit back to back at the maximum rate the
//! wire allows while escalating load runs beside them. The question is whether
//! a refill interrupt at esp-hal's highest dispatchable priority (level 3 on
//! Xtensa) ever arrives late enough for the transmitter to reach its guard
//! word — because if it does, the fix is a level-4/5 assembly shim, which is a
//! whole follow-up plan.
//!
//! # How the load and the LEDs overlap
//!
//! Frames are started from thread context, as in every other build here — the
//! refill path under test must be the deployment path. The trick is that the
//! load generator runs *between* frame batches while the previous batch is
//! still being serviced by the radio's own tasks and interrupts: a WiFi scan
//! is polled a step at a time rather than waited on, and an ESP-NOW send is
//! issued and left to the driver. So the strips never stop for the load, and
//! the load never stops for the strips.
//!
//! The frame buffers are `static` and never written after compile time, which
//! is what makes handing their addresses to the handler sound with no `unsafe`
//! beyond `start_frame`'s own contract.
//!
//! # Scenarios
//!
//! | id | load |
//! |----|------|
//! | S0 | idle — no radio, no logging beyond the per-second report |
//! | S1 | logging load: verbose serial output between frames |
//! | S2 | WiFi scan loop (STA, repeated active scans) |
//! | S3 | ESP-NOW broadcast spam (lp2025's actual radio usage) |
//! | S4 | STA + traffic, only if an AP is configured at build time |
//!
//! S0 and S1 run *before* the radio stack is initialised, so S0 really is the
//! quiet baseline the later phases are compared against.
//!
//! # What S2 does and does not prove
//!
//! S2 reports `aps_total` but never asserts on it, because on the board this
//! was developed against that number is always zero: a 13-channel passive
//! sweep at a second per channel turned up a single beacon at −96 dBm, while
//! an ESP32-C6 beside it sees three to seven access points through the same
//! driver with the same scan configuration. That is a receive path roughly 40 dB
//! down — a property of the board, not of this firmware, and not one the
//! firmware can correct.
//!
//! It does not weaken the scenario. What S2 exists to impose is the *load* of a
//! scan, and that load does not depend on anyone answering: `scans` and the
//! ~155 ms each one takes show the radio hopping thirteen channels,
//! transmitting a probe request on every one and keeping the receiver hot the
//! whole time, roughly six times a second for the length of the scenario.
//!
//! # Output
//!
//! ```text
//! E6: MEASURE scenario=S2 ch=0 leds=300 frames=6180 complete=6180 truncated=0 \
//!     trips=0 skips=1 errors=0 lag_avg=4.1 lag_max=11 half=24 \
//!     lag_hist=5900,240,30,10,0,0,0,0,0
//! E6: MEASURE scenario=S2 scans=3870 aps_total=0 scan_errors=0 scan_deadlines=0
//! E6: RESULT scenario=S2 seconds=600 frames=61800 truncated=0 lag_max=11 half=24
//! ```
//!
//! Each load also reports what it managed to do — `scans` for S2, `sends` for
//! S3 — so that a scenario which never generated any load cannot be mistaken
//! for one that generated load without disturbing the strips.
//!
//! `lag_hist` has nine buckets: eight covering the deadline (one ping-pong
//! half) in eighths, then everything at or past the half — see
//! [`lp_ws281x::LAG_BUCKETS`]. The last bucket is the interesting one.

use esp_hal::peripherals::Peripherals;
use esp_hal::rmt::{Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::gpio::Level;
use esp_hal::time::{Duration, Instant};

use lp_ws281x::{ChannelTiming, ColorOrder, LAG_BUCKETS};

use crate::s3_rmt::{self, BLOCKS_PER_CHANNEL, TX_BLOCKS, TX_CHANNELS};
use crate::{install_isr, DRIVER, RMT_CLOCK};

/// LEDs per channel. Long strips on every channel: a 300-LED frame is 7200
/// bits, i.e. 300 refills, and four of them together keep the handler busy
/// ~15 % of the time before any radio exists. The lengths are unequal so the
/// four channels' half-boundary crossings drift through every phase relation
/// instead of locking into one.
pub const STRIP_LEDS: [usize; TX_CHANNELS] = [300, 256, 200, 150];

/// The longest strip, in bytes — the size of every channel's static buffer.
const MAX_FRAME_BYTES: usize = 300 * 3;

/// How long each scenario runs. The plan asks for ≥10 minutes per scenario;
/// `STRESS_SECONDS=30 cargo build …` shortens it for a smoke run.
const SCENARIO_SECS: u64 = match option_env!("STRESS_SECONDS") {
    Some(s) => parse_u64(s),
    None => 600,
};

/// Decimal `u64` parser for [`SCENARIO_SECS`]; a malformed value is a build
/// failure, not a silently wrong soak length.
const fn parse_u64(s: &str) -> u64 {
    let bytes = s.as_bytes();
    let mut i = 0;
    let mut acc = 0u64;
    while i < bytes.len() {
        let d = bytes[i];
        assert!(d.is_ascii_digit(), "STRESS_SECONDS must be a decimal integer");
        acc = acc * 10 + (d - b'0') as u64;
        i += 1;
    }
    assert!(acc > 0, "STRESS_SECONDS must be positive");
    acc
}

/// How often a `MEASURE` block is printed.
const REPORT_INTERVAL: Duration = Duration::from_millis(10_000);

/// Lines of chatter per burst in S1, and how often a burst happens.
const LOG_BURST_LINES: usize = 20;
const LOG_BURST_INTERVAL: Duration = Duration::from_millis(50);

/// ESP-NOW payload size for S3. 250 bytes is the protocol maximum, i.e. the
/// most airtime per send.
const ESP_NOW_PAYLOAD: usize = 250;

/// Sends per burst in S3. Eight 250-byte broadcasts back to back is as hard as
/// the sender can be driven from thread context without abandoning the strips.
const ESP_NOW_BURST: usize = 8;

/// How long S2 waits for one scan before giving up on it. A default active scan
/// covers 13 channels at ~10 ms each and returns in ~155 ms; anything past five
/// seconds is not a slow scan, it is a `ScanDone` this loop will never see.
const SCAN_DEADLINE: Duration = Duration::from_millis(5_000);

/// How long S2 keeps transmitting frames after abandoning a scan, so the
/// hardware scan already on the air can finish before the next one starts.
const SCAN_SETTLE: Duration = Duration::from_millis(1_000);

/// One `L2:` line per this many scans. A 600 s scenario completes a scan every
/// ~155 ms, and 3800 lines of scan chatter would be a logging load, not a
/// record of one.
const SCAN_LOG_EVERY: u32 = 25;

/// The frames the harness transmits, compiled into flash and never modified.
///
/// A deterministic gradient rather than zeros: an all-zero frame is a wall of
/// short pulses, which is the *easiest* case for the refill path (a zero bit
/// and a one bit take the same 1.25 µs on the wire, but a scope trace of a
/// varied pattern is readable, and a strip on the pin shows something).
static FRAMES: [[u8; MAX_FRAME_BYTES]; TX_CHANNELS] = build_frames();

const fn build_frames() -> [[u8; MAX_FRAME_BYTES]; TX_CHANNELS] {
    let mut frames = [[0u8; MAX_FRAME_BYTES]; TX_CHANNELS];
    let mut ch = 0;
    while ch < TX_CHANNELS {
        let mut i = 0;
        while i < MAX_FRAME_BYTES {
            // A per-channel phase shift keeps the four wires visibly distinct.
            frames[ch][i] = ((i * 7 + ch * 53) % 256) as u8;
            i += 1;
        }
        ch += 1;
    }
    frames
}

/// Which load is running.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    S0Idle,
    S1Logging,
    S2WifiScan,
    S3EspNow,
}

impl Scenario {
    fn id(self) -> &'static str {
        match self {
            Scenario::S0Idle => "S0",
            Scenario::S1Logging => "S1",
            Scenario::S2WifiScan => "S2",
            Scenario::S3EspNow => "S3",
        }
    }
}

pub fn run(peripherals: Peripherals) -> ! {
    esp_println::println!(
        "led-lab-esp32s3: P6 stress harness, {} channels on GPIO4-7, strips {:?}",
        TX_CHANNELS,
        STRIP_LEDS,
    );

    // The radio stack needs a heap and a scheduler; both are declared up front
    // so that S0/S1 run in exactly the memory layout S2/S3 will.
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(
        peripherals.SW_INTERRUPT,
    );
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut rmt = match Rmt::new(peripherals.RMT, RMT_CLOCK) {
        Ok(rmt) => rmt,
        Err(e) => {
            esp_println::println!("E6: FAIL stress_s3 reason=rmt_init:{e:?}");
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
    // Kept alive for the whole run: dropping one would release its memory
    // block, disconnect its pin, and (with the last one) gate the RMT clock.
    let _channels = match (
        rmt.channel0.configure_tx(&config),
        rmt.channel1.configure_tx(&config),
        rmt.channel2.configure_tx(&config),
        rmt.channel3.configure_tx(&config),
    ) {
        (Ok(c0), Ok(c1), Ok(c2), Ok(c3)) => [
            c0.with_pin(peripherals.GPIO4),
            c1.with_pin(peripherals.GPIO5),
            c2.with_pin(peripherals.GPIO6),
            c3.with_pin(peripherals.GPIO7),
        ],
        _ => {
            esp_println::println!("E6: FAIL stress_s3 reason=configure_tx");
            crate::halt();
        }
    };

    s3_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

    let timings = [
        ChannelTiming::WS2812,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Rgb),
        ChannelTiming::WS2811,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Bgr),
    ];
    for (ch, timing) in timings.iter().enumerate() {
        if let Err(e) = DRIVER.configure_default_clock(ch as u8, timing) {
            esp_println::println!("E6: FAIL stress_s3 reason=configure_ch{ch}:{e:?}");
            crate::halt();
        }
    }

    // --- S0 and S1 run before the radio exists, so the baseline is real. ---
    {
        let mut soak = Soak::start(Scenario::S0Idle);
        while !soak.expired() {
            soak.frame_batch();
        }
        soak.finish();
    }
    {
        let mut soak = Soak::start(Scenario::S1Logging);
        while !soak.expired() {
            soak.frame_batch();
            log_burst();
        }
        soak.finish();
    }

    // --- Bring the radio up. ---
    let (mut controller, interfaces) =
        match esp_radio::wifi::new(peripherals.WIFI, Default::default()) {
            Ok(pair) => pair,
            Err(e) => {
                esp_println::println!("E6: FAIL stress_s3 reason=wifi_new:{e:?}");
                crate::halt();
            }
        };
    let station_config = esp_radio::wifi::sta::StationConfig::default();
    if let Err(e) = controller.set_config(&esp_radio::wifi::Config::Station(station_config)) {
        esp_println::println!("E6: FAIL stress_s3 reason=wifi_set_config:{e:?}");
        crate::halt();
    }
    let mac = interfaces.station.mac_address();
    esp_println::println!(
        "E6: MEASURE radio=up mac={:02x}{:02x}{:02x}{:02x}{:02x}{:02x} free={} used={}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        esp_alloc::HEAP.free(),
        esp_alloc::HEAP.used(),
    );

    // A scan needs a real receiver, and this board's is deaf: a full 13-channel
    // passive sweep at 1 s per channel found one beacon at -96 dBm, and the
    // ESP32-C6 sitting beside it sees three to seven APs with this same driver
    // and scan configuration. So `aps` is reported but never asserted on — what
    // S2 is actually here to measure is the *radio load* a scan imposes, and
    // that load is real whether or not anything answers: 13 channels, a probe
    // request transmitted on each, and the receiver hot throughout.
    //
    // The scan is *polled* between frame batches rather than waited on, so the
    // strips keep running while the radio scans.
    //
    // The deadline below is the reason this loop cannot go quiet. `scan_async`
    // waits for a `ScanDone` on esp-radio's internal pub-sub channel, which
    // holds two messages and drops a lagging subscriber's oldest silently
    // (`WaitResult::Lagged(_) => continue` in embassy-sync). A `ScanDone` that
    // is evicted before this loop polls for it is gone, and the future then
    // waits forever for an event that will never be published again — no
    // output, no panic, just a scenario that stops reporting. Dropping the
    // future on a deadline frees the AP list and the subscriber and turns that
    // hang into a counter.
    {
        let mut soak = Soak::start(Scenario::S2WifiScan);
        let config = esp_radio::wifi::scan::ScanConfig::default();
        let mut scans = 0u32;
        let mut aps_total = 0u32;
        let mut errors = 0u32;
        let mut deadlines = 0u32;
        while !soak.expired() {
            let started = Instant::now();
            let mut scan = core::pin::pin!(controller.scan_async(&config));
            loop {
                soak.frame_batch();
                match poll_once(scan.as_mut()) {
                    Some(Ok(aps)) => {
                        scans += 1;
                        aps_total += aps.len() as u32;
                        if scans % SCAN_LOG_EVERY == 0 {
                            esp_println::println!(
                                "L2: scan n={scans} aps={} ms={} aps_total={aps_total} \
                                 errors={errors} deadlines={deadlines}",
                                aps.len(),
                                started.elapsed().as_millis(),
                            );
                        }
                        break;
                    }
                    Some(Err(e)) => {
                        errors += 1;
                        esp_println::println!("L2: scan_err n={scans} errors={errors} {e:?}");
                        break;
                    }
                    None => {}
                }
                if started.elapsed() > SCAN_DEADLINE {
                    deadlines += 1;
                    esp_println::println!(
                        "L2: scan_deadline n={scans} deadlines={deadlines} ms={}",
                        started.elapsed().as_millis()
                    );
                    break;
                }
                if soak.expired() {
                    break;
                }
            }
            // Only a deadline can leave the inner loop this late: a completed
            // scan breaks well before it, and so does an error.
            if started.elapsed() > SCAN_DEADLINE {
                // The hardware scan this loop walked away from is still on the
                // air; starting another immediately would only earn an error.
                // Frames keep going out while it drains.
                let settle = Instant::now();
                while settle.elapsed() < SCAN_SETTLE && !soak.expired() {
                    soak.frame_batch();
                }
            }
        }
        esp_println::println!(
            "E6: MEASURE scenario=S2 scans={scans} aps_total={aps_total} scan_errors={errors} \
             scan_deadlines={deadlines}"
        );
        soak.finish();
    }

    // S3: ESP-NOW broadcast spam. `send` hands the frame to the driver and the
    // wait is short, so a burst fits between frame batches without ever
    // stopping the strips for long.
    //
    // `sends` and `send_errors` are both reported because "ran clean" and "never
    // transmitted" produce the same silence otherwise: the send path fails by
    // returning an error, not by printing anything.
    {
        let mut esp_now = interfaces.esp_now;
        let mut soak = Soak::start(Scenario::S3EspNow);
        let mut sends = 0u32;
        let mut send_errors = 0u32;
        let mut wait_errors = 0u32;
        while !soak.expired() {
            soak.frame_batch();
            esp_now_burst(&mut esp_now, &mut sends, &mut send_errors, &mut wait_errors);
        }
        esp_println::println!(
            "E6: MEASURE scenario=S3 sends={sends} bytes_per_send={ESP_NOW_PAYLOAD} \
             send_errors={send_errors} wait_errors={wait_errors}"
        );
        soak.finish();
    }

    // S4 needs an access point; there is no way to invent one.
    match option_env!("LED_LAB_WIFI_SSID") {
        Some(ssid) => esp_println::println!(
            "E6: SKIP scenario=S4 reason=udp_stack_absent ssid_configured={ssid}"
        ),
        None => esp_println::println!("E6: SKIP scenario=S4 reason=no_ap_configured"),
    }

    esp_println::println!("E6: DONE stress_s3 scenarios=S0,S1,S2,S3");
    crate::halt()
}

/// One scenario's run: resets the counters, drives frame batches, reports.
struct Soak {
    scenario: Scenario,
    started: Instant,
    last_report: Instant,
    limit: Duration,
    /// Frame batches that did not complete within [`crate::FRAME_TIMEOUT`].
    timeouts: usize,
}

impl Soak {
    fn start(scenario: Scenario) -> Self {
        for ch in 0..TX_CHANNELS {
            if let Some(state) = DRIVER.channel(ch as u8) {
                state.reset_stats();
            }
        }
        esp_println::println!(
            "E6: START scenario={} seconds={}",
            scenario.id(),
            SCENARIO_SECS
        );
        Self {
            scenario,
            started: Instant::now(),
            last_report: Instant::now(),
            limit: Duration::from_millis(SCENARIO_SECS * 1000),
            timeouts: 0,
        }
    }

    fn expired(&self) -> bool {
        self.started.elapsed() >= self.limit
    }

    /// Transmit one frame on every channel, simultaneously, and wait for the
    /// set to finish — ~9.4 ms for a 300-LED strip, ~1200 refill interrupts.
    fn frame_batch(&mut self) {
        let starts: [(u8, &[u8]); TX_CHANNELS] = [
            (0, &FRAMES[0][..STRIP_LEDS[0] * 3]),
            (1, &FRAMES[1][..STRIP_LEDS[1] * 3]),
            (2, &FRAMES[2][..STRIP_LEDS[2] * 3]),
            (3, &FRAMES[3][..STRIP_LEDS[3] * 3]),
        ];
        let round = Instant::now();
        let mut timed_out = false;
        let send = DRIVER.send_blocking_all(&starts, || {
            if round.elapsed() > crate::FRAME_TIMEOUT {
                timed_out = true;
                for ch in 0..TX_CHANNELS {
                    DRIVER.abort(ch as u8);
                }
            }
        });
        if timed_out {
            self.timeouts += 1;
        }
        if let Err(e) = send {
            esp_println::println!("E6: FAIL stress_s3 reason=start:{e:?}");
        }
        if self.last_report.elapsed() >= REPORT_INTERVAL {
            report(self.scenario, self.started.elapsed().as_secs(), self.timeouts);
            self.last_report = Instant::now();
        }
    }

    fn finish(self) {
        report(self.scenario, self.started.elapsed().as_secs(), self.timeouts);

        let mut frames = 0usize;
        let mut truncated = 0usize;
        let mut lag_max = 0i32;
        let mut errors = 0usize;
        let mut over_half = 0u32;
        for ch in 0..TX_CHANNELS {
            let stats = DRIVER.stats(ch as u8);
            frames += stats.frames;
            truncated += stats.guard_trips;
            errors += stats.errors;
            over_half += stats.lag_over_half();
            if stats.refill_lag_max > lag_max {
                lag_max = stats.refill_lag_max;
            }
        }
        let half = DRIVER.channel(0).map(|c| c.half_words()).unwrap_or(0);
        esp_println::println!(
            "E6: RESULT scenario={} seconds={} frames={} truncated={} errors={} \
             timeouts={} lag_max={} lag_over_half={} half={}",
            self.scenario.id(),
            self.started.elapsed().as_secs(),
            frames,
            truncated,
            errors,
            self.timeouts,
            lag_max,
            over_half,
            half,
        );
    }
}

/// Poll a future once without blocking; `None` means still pending.
fn poll_once<F: core::future::Future>(
    fut: core::pin::Pin<&mut F>,
) -> Option<F::Output> {
    use core::task::{Context, Poll};
    let waker = noop_waker();
    let mut cx = Context::from_waker(&waker);
    match fut.poll(&mut cx) {
        Poll::Ready(out) => Some(out),
        Poll::Pending => None,
    }
}

/// One `MEASURE` line per channel.
fn report(scenario: Scenario, secs: u64, timeouts: usize) {
    for ch in 0..TX_CHANNELS {
        let stats = DRIVER.stats(ch as u8);
        let half = DRIVER
            .channel(ch as u8)
            .map(|c| c.half_words())
            .unwrap_or(0);
        let (lag_int, lag_frac) = mean_lag_tenths(stats.refill_lag_sum, stats.refill_lag_count);
        esp_println::println!(
            "E6: MEASURE scenario={} secs={} ch={} leds={} frames={} complete={} \
             truncated={} skips={} errors={} timeouts={} lag_avg={}.{} lag_max={} \
             half={} refills={} lag_hist={}",
            scenario.id(),
            secs,
            ch,
            STRIP_LEDS[ch],
            stats.frames,
            stats.complete_frames(),
            stats.guard_trips,
            stats.guard_skips,
            stats.errors,
            timeouts,
            lag_int,
            lag_frac,
            stats.refill_lag_max,
            half,
            stats.refill_lag_count,
            HistFmt(stats.lag_hist),
        );
    }
}

/// Comma-separated histogram, without dragging a formatter machinery in.
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

fn mean_lag_tenths(sum: i32, count: i32) -> (i32, i32) {
    if count == 0 {
        return (0, 0);
    }
    let tenths = sum.saturating_mul(10) / count;
    (tenths / 10, (tenths % 10).abs())
}

/// S1's load: a burst of serial chatter, which is what the lp2025 comment
/// blamed for its dropped frames.
fn log_burst() {
    static mut SEQ: u32 = 0;
    let last = Instant::now();
    // SAFETY: single-threaded thread context; the counter is never touched by
    // an interrupt handler.
    let seq = unsafe {
        SEQ = SEQ.wrapping_add(1);
        SEQ
    };
    for line in 0..LOG_BURST_LINES {
        esp_println::println!(
            "L1: chatter seq={seq} line={line} \
             payload=the-quick-brown-fox-jumps-over-the-lazy-dog-0123456789"
        );
    }
    while last.elapsed() < LOG_BURST_INTERVAL {}
}

/// S3's load: broadcast ESP-NOW frames back to back at the maximum payload.
///
/// A completed `wait` means the driver reported the frame off the antenna, so
/// `sends` is a transmission count and not merely a call count. Errors are
/// counted rather than only printed: at this rate a printed error would itself
/// become the load.
fn esp_now_burst(
    esp_now: &mut esp_radio::esp_now::EspNow<'_>,
    sends: &mut u32,
    send_errors: &mut u32,
    wait_errors: &mut u32,
) {
    let payload = [0xA5u8; ESP_NOW_PAYLOAD];
    for _ in 0..ESP_NOW_BURST {
        match esp_now.send(&esp_radio::esp_now::BROADCAST_ADDRESS, &payload) {
            Ok(waiter) => {
                if let Err(e) = waiter.wait() {
                    *wait_errors += 1;
                    if *wait_errors % 100 == 1 {
                        esp_println::println!("L3: espnow_err n={} {e:?}", *wait_errors);
                    }
                } else {
                    *sends += 1;
                }
            }
            Err(e) => {
                *send_errors += 1;
                if *send_errors % 100 == 1 {
                    esp_println::println!("L3: espnow_send_err n={} {e:?}", *send_errors);
                }
                // A full queue is normal at this rate; back off a touch rather
                // than spinning on the error path.
                let t = Instant::now();
                while t.elapsed() < Duration::from_millis(2) {}
            }
        }
    }
}

fn noop_waker() -> core::task::Waker {
    use core::task::{RawWaker, RawWakerVTable, Waker};
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(core::ptr::null(), &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    // SAFETY: every vtable entry is a no-op on a null data pointer, which is
    // the standard no-op waker construction; nothing is ever dereferenced.
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
}
