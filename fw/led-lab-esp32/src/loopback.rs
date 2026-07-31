//! P5 — classic-ESP32 four-channel RMT loopback self-test: the no-oscilloscope
//! timing oracle, ported from the S3 harness.
//!
//! Routes each of the four TX waveforms (GPIO16/17/18/19) into an RMT **RX
//! channel** through the GPIO matrix — no wires, no strips required — captures
//! every (level, duration) pair at 12.5 ns resolution, and asserts the WS281x
//! wire protocol numerically while all four channels transmit simultaneously:
//!
//! * every bit classifies cleanly as 0/1 and the decoded bytes equal the sent
//!   bytes, per channel, under four different configurations (WS2812/GRB,
//!   WS2812/RGB, **WS2811** timing, WS2812/BGR);
//! * per-bit high time and period sit within ±2 ticks (±25 ns) of *that
//!   channel's* configuration;
//! * no channel ever decodes to another channel's pattern (cross-talk);
//! * the trailing low is at least the configured 300 µs latch (as far as the
//!   idle-threshold capture can bound it);
//! * a 100-frame concurrent soak decodes clean on all four channels with zero
//!   guard trips — four transmitters running together every round, with the
//!   witnessed channel's receiver armed **one at a time**. Arming all four
//!   receivers at once is what made this assertion red for a while: two or
//!   more concurrent RMT RX channels corrupt what the transmitters put on the
//!   wire (confirmed on the pad by a non-RMT CPU witness — see `src/diag.rs`
//!   and the README). The transmitters were never at fault;
//! * a threshold interrupt suppressed on **one** channel truncates that
//!   channel at exactly its guard word while the others, sharing the same
//!   handler entries, finish their frames intact.
//!
//! # Signal routing (option 1: `Flex::split`, same as the S3)
//!
//! [`Flex::new`]`(GPIOn).split()` yields a frozen (input, output) signal pair
//! for the same pad; the output half drives the RMT TX channel, the input half
//! feeds the paired RX channel through the GPIO matrix. The esp-hal gpio
//! interconnect API is chip-generic and carries to the classic ESP32 unchanged.
//!
//! # RX capacity — the classic constraint (`rmt.has_rx_wrap` = false)
//!
//! The classic ESP32 has **no RX wrap**: a receiver stops writing at the end
//! of its memory window, and esp-hal rejects a capture buffer larger than the
//! window outright. With four TX + four RX channels taking one 64-word block
//! each, a routine capture is at most 64 items = 64 bits. The routine test
//! frames (1–2 LEDs = 24–48 bits) fit; the truncation test does not — its
//! victim stops at bit 96 (one 64-bit window + one 32-bit half) — so the
//! truncation phase **reconfigures the peripheral**: victim's receiver gets 2
//! memory blocks (128 items) by sacrificing one bystander's receiver. That is
//! the one structural divergence from the S3 harness, where RX wrap let a
//! 48-item window fill a 96-item buffer. See [`run`].
//!
//! # RX side
//!
//! Same 80 MHz clock, divider 1 (12.5 ns ticks). The input filter is **on**
//! here ([`RX_FILTER_TICKS`]) where the S3 and C6 harnesses leave it off —
//! four adjacent GPIOs switching together glitch a wrap-less receiver that has
//! no filter. Idle threshold 30 000 ticks
//! (375 µs) — above the 300 µs latch, so the capture is only terminated by
//! genuine end-of-frame idle. (The classic `idle_thres` field is 16 bits
//! against the S3's 15 — either way 30 000 fits.) The esp-hal blocking RX
//! transaction polls `INT_RAW` directly, so it coexists with this firmware's
//! own TX interrupt handler (which enables and consumes only the TX causes:
//! note the classic's `chN_err` is a *combined* TX/RX error bit, so keeping
//! `INT_ENA` clear for RX channels is what keeps RX errors out of the
//! driver's snapshot).

use esp_hal::gpio::{Flex, Level};
use esp_hal::peripherals::Peripherals;
use esp_hal::rmt::{
    Channel, PulseCode, Rmt, Rx, RxChannelConfig, RxChannelCreator, TxChannelConfig,
    TxChannelCreator,
};
use esp_hal::time::{Duration, Instant};
use esp_hal::Blocking;

use lp_ws281x::{ChannelTiming, ColorOrder, PulseCodes, PulseItem};

use crate::esp32_rmt::{self, BLOCKS_PER_CHANNEL, TX_BLOCKS};
use crate::{install_isr, DRIVER, FRAME_TIMEOUT, RMT_CLOCK};

/// TX channels under test, each paired with RX channel `ch + 4` in the main
/// phase.
const CHANNELS: usize = 4;

/// RX memory blocks per receive channel in the main phase. Four receivers,
/// four spare blocks: one each.
const RX_BLOCKS: u8 = 1;

/// Capture buffer size in RMT items for the main phase. **Exactly one RX
/// window** — the classic has no RX wrap, so the window is the hard ceiling
/// (esp-hal returns `InvalidDataLength` for anything bigger).
const RX_CODES: usize = 64;

/// Victim capture buffer in the truncation phase: two blocks' worth.
const TRUNC_RX_CODES: usize = 2 * RX_CODES;

/// RX idle threshold in 12.5 ns ticks: 375 µs. Longer than the 300 µs latch,
/// far longer than any in-frame low (≤ 950 ns), and within the classic's
/// 16-bit `idle_thres` field.
const IDLE_THRESHOLD_TICKS: u16 = 30_000;

/// RX input filter width in 12.5 ns ticks: pulses shorter than this are
/// ignored by the receiver.
///
/// **Not optional on the classic ESP32.** With the filter off (as on the S3
/// and C6, where it is harmless), four adjacent GPIOs switching in lockstep
/// inject simultaneous-switching glitches that the wrap-less receiver records
/// as extra edges, and the capture — not the wire — is what fails. 15 ticks
/// (187 ns) is below the shortest legitimate high time in this suite
/// (WS2811's T0H = 300 ns = 24 ticks) and well above the glitches. The
/// `filter_thres` field is 8 bits, so 255 ticks is the ceiling.
const RX_FILTER_TICKS: u8 = 15;

/// Timing tolerance in ticks: ±25 ns at 12.5 ns per tick — the S3 measured
/// every bit exactly nominal at this resolution, and the classic is asserted
/// to the same bound. A miss here is a finding, not a tolerance problem.
const TOL_TICKS: u16 = 2;

/// Frames in the concurrent soak run.
const SOAK_FRAMES: usize = 100;

/// The TX channel whose threshold interrupt the truncation test suppresses
/// (the WS2811-timing channel, as on the S3).
const VICTIM: u8 = 2;

/// The truncation test's frame: 16 LEDs = 384 bits, many refills long.
const TRUNC_LEDS: usize = 16;

/// Where the truncation must stop: prefill (64 bits — one full window) plus
/// the one refill that *is* serviced (32 bits — one half), then the
/// transmitter walks into the guard planted by that refill. The classic's
/// 64-word blocks make this 96 against the S3's 72. Mirrored by
/// `lp-ws281x/tests/hooks.rs` on the host (which parameterises by half size).
const TRUNC_EXPECT_BITS: usize = 96;

/// Upper bound on distinct bits in one capture (the victim's two-block
/// buffer is the largest).
const MAX_BITS: usize = TRUNC_RX_CODES;

/// Longest frame any subtest transmits, in bytes — sizes the expected-bytes
/// scratch buffers.
const MAX_FRAME_BYTES: usize = TRUNC_LEDS * 3;

/// LEDs per channel in the routine captures. Two lengths, so the four
/// channels stop at two different moments and a `tx_end` for one lands in the
/// same interrupt snapshot as a refill for another. 2 LEDs = 48 bits also
/// fills a wrap-less 64-item receiver comfortably.
const TEST_LEDS: [usize; CHANNELS] = [2, 1, 2, 1];

/// The known-answer frames: byte values exercising both edges of each byte
/// and plenty of bit transitions, distinct per channel so a channel emitting
/// its neighbour's pixels is a decode failure and not a coincidence.
const TEST_FRAMES: [[u8; 6]; CHANNELS] = [
    [0xA5, 0x3C, 0x0F, 0x01, 0x80, 0xFF],
    [0x5A, 0xC3, 0xF0, 0x00, 0x00, 0x00],
    [0x11, 0x22, 0x44, 0x88, 0xEE, 0x77],
    [0xFE, 0x01, 0x7F, 0x00, 0x00, 0x00],
];

type RxCh<'ch> = Channel<'ch, Blocking, Rx>;

/// Per-channel wire configuration. Channel 0 keeps the exact WS2812/GRB setup
/// the golden vectors in `lp-ws281x/tests/golden/` are captured with.
fn channel_timings() -> [ChannelTiming; CHANNELS] {
    [
        ChannelTiming::WS2812,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Rgb),
        ChannelTiming::WS2811,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Bgr),
    ]
}

/// Nominal per-bit tick values, decoded from the same [`PulseCodes`] the
/// driver transmits — the oracle and the transmitter cannot disagree about
/// what was configured.
struct Nominal {
    t0h: u16,
    t0l: u16,
    t1h: u16,
    t1l: u16,
    /// Bits with a high time at or above this are ones.
    mid: u16,
    /// Full latch duration in ticks.
    latch: u32,
}

impl Nominal {
    fn from_timing(timing: &ChannelTiming) -> Self {
        // The encoder was validated on the host; unwraps cannot fire for these
        // constants, and a panic here would print an E5 FAIL anyway.
        let codes = PulseCodes::at_default_clock(timing).unwrap();
        let zero = PulseItem::decode(codes.zero).unwrap();
        let one = PulseItem::decode(codes.one).unwrap();
        let latch = PulseItem::decode(codes.latch).unwrap();
        Self {
            t0h: zero.first.ticks,
            t0l: zero.second.ticks,
            t1h: one.first.ticks,
            t1l: one.second.ticks,
            mid: (zero.first.ticks + one.first.ticks) / 2,
            latch: latch.first.ticks as u32 + latch.second.ticks as u32,
        }
    }
}

/// One capture, folded into per-bit (high, low) tick pairs.
struct Bits {
    high: [u16; MAX_BITS],
    low: [u32; MAX_BITS],
    len: usize,
    /// Low ticks recorded between RX start and the first rising edge — an
    /// artifact of starting the receiver early, not part of the waveform.
    leading_low: u32,
    /// The low run after the final bit's high: last bit low + latch + idle,
    /// as far as the idle threshold lets the receiver see.
    trailing_low: u32,
    /// True when the capture ended in a high level or a zero-duration marker
    /// immediately after one — no trailing low was recorded at all.
    ended_high: bool,
}

impl Bits {
    const fn new() -> Self {
        Self {
            high: [0; MAX_BITS],
            low: [0; MAX_BITS],
            len: 0,
            leading_low: 0,
            trailing_low: 0,
            ended_high: false,
        }
    }
}

/// Iterate the (level, ticks) halves of captured items, fusing at the first
/// zero-duration half (the hardware's end marker).
struct Halves<'a> {
    codes: &'a [PulseCode],
    idx: usize,
    second: bool,
}

impl<'a> Halves<'a> {
    fn new(codes: &'a [PulseCode]) -> Self {
        Self {
            codes,
            idx: 0,
            second: false,
        }
    }
}

impl Iterator for Halves<'_> {
    type Item = (bool, u16);

    fn next(&mut self) -> Option<(bool, u16)> {
        let code = *self.codes.get(self.idx)?;
        let (level, ticks) = if self.second {
            (code.level2(), code.length2())
        } else {
            (code.level1(), code.length1())
        };
        if self.second {
            self.idx += 1;
        }
        self.second = !self.second;
        if ticks == 0 {
            // End marker: fuse the iterator.
            self.idx = self.codes.len();
            return None;
        }
        Some((matches!(level, Level::High), ticks))
    }
}

/// Fold captured items into bits: skip the leading low, then pair each high
/// run with the low run that follows it. Consecutive same-level halves (which
/// the receiver does not normally produce) are merged defensively.
fn parse(codes: &[PulseCode], out: &mut Bits) -> Result<(), &'static str> {
    *out = Bits::new();
    let mut started = false;
    let mut in_high = false;
    let mut high_acc: u32 = 0;
    let mut low_acc: u32 = 0;

    for (level, ticks) in Halves::new(codes) {
        let ticks = ticks as u32;
        if !started {
            if !level {
                out.leading_low += ticks;
                continue;
            }
            started = true;
            in_high = true;
            high_acc = ticks;
            continue;
        }
        match (level, in_high) {
            (true, true) => high_acc += ticks,
            (false, true) => {
                in_high = false;
                low_acc = ticks;
            }
            (false, false) => low_acc += ticks,
            (true, false) => {
                // A rising edge closes the previous bit.
                if out.len >= MAX_BITS {
                    return Err("too_many_bits");
                }
                out.high[out.len] = high_acc.min(u16::MAX as u32) as u16;
                out.low[out.len] = low_acc;
                out.len += 1;
                in_high = true;
                high_acc = ticks;
            }
        }
    }

    if started {
        if out.len >= MAX_BITS {
            return Err("too_many_bits");
        }
        out.high[out.len] = high_acc.min(u16::MAX as u32) as u16;
        if in_high {
            // The receiver ended the item list right after the final high: the
            // over-threshold trailing low was recorded as a zero-duration
            // marker, not as a measured duration. `low_acc` still holds the
            // previous bit's low, so it must not leak into this one.
            out.low[out.len] = 0;
            out.ended_high = true;
            out.trailing_low = 0;
        } else {
            out.low[out.len] = low_acc;
            out.trailing_low = low_acc;
        }
        out.len += 1;
    }
    Ok(())
}

/// The wire byte order for `frame` under `order` — what a strip (and
/// therefore the receiver) sees. Returns the byte count.
fn wire_bytes(frame: &[u8], order: ColorOrder, out: &mut [u8]) -> usize {
    let mut n = 0;
    for pixel in frame.chunks_exact(3) {
        for slot in 0..3 {
            if n < out.len() {
                out[n] = pixel[order.source_index(slot)];
            }
            n += 1;
        }
    }
    n.min(out.len())
}

/// Classify each bit by its high time and pack MSB-first into bytes. Returns
/// the byte count, or `Err` if the bit count is not a whole number of bytes.
fn decode(bits: &Bits, mid: u16, out: &mut [u8]) -> Result<usize, &'static str> {
    if bits.len % 8 != 0 {
        return Err("bit_count_not_byte_aligned");
    }
    let bytes = bits.len / 8;
    if bytes > out.len() {
        return Err("too_many_bytes");
    }
    for (i, byte) in out.iter_mut().take(bytes).enumerate() {
        let mut b = 0u8;
        for bit in 0..8 {
            b <<= 1;
            if bits.high[i * 8 + bit] >= mid {
                b |= 1;
            }
        }
        *byte = b;
    }
    Ok(bytes)
}

/// Min/max timing actuals over one capture, classified against `Nominal`.
struct TimingStats {
    t0h_min: u16,
    t0h_max: u16,
    t1h_min: u16,
    t1h_max: u16,
    period_min: u32,
    period_max: u32,
    zeros: usize,
    ones: usize,
    /// Index of the first bit outside tolerance, if any.
    violation: Option<usize>,
}

fn timing_stats(bits: &Bits, nom: &Nominal) -> TimingStats {
    let mut s = TimingStats {
        t0h_min: u16::MAX,
        t0h_max: 0,
        t1h_min: u16::MAX,
        t1h_max: 0,
        period_min: u32::MAX,
        period_max: 0,
        zeros: 0,
        ones: 0,
        violation: None,
    };
    let tol = TOL_TICKS as u32;
    for i in 0..bits.len {
        let h = bits.high[i];
        let one = h >= nom.mid;
        let (h_nom, p_nom) = if one {
            s.ones += 1;
            s.t1h_min = s.t1h_min.min(h);
            s.t1h_max = s.t1h_max.max(h);
            (nom.t1h, nom.t1h as u32 + nom.t1l as u32)
        } else {
            s.zeros += 1;
            s.t0h_min = s.t0h_min.min(h);
            s.t0h_max = s.t0h_max.max(h);
            (nom.t0h, nom.t0h as u32 + nom.t0l as u32)
        };
        let mut bad = (h as u32).abs_diff(h_nom as u32) > tol;
        // The final bit's low merges into the latch; its period is asserted
        // via the trailing-low check instead.
        if i + 1 < bits.len {
            let period = h as u32 + bits.low[i];
            s.period_min = s.period_min.min(period);
            s.period_max = s.period_max.max(period);
            bad |= period.abs_diff(p_nom) > tol;
        }
        if bad && s.violation.is_none() {
            s.violation = Some(i);
        }
    }
    s
}

/// Transmit on every driver channel at once while `K` receivers capture; hand
/// back the receivers and the item counts.
///
/// Generic over the receiver count because the truncation phase runs with
/// three receivers of unequal buffer sizes (see module docs) while the main
/// phase runs with four equal ones. The buffers must each fit their channel's
/// RX window — the classic has no RX wrap, so there is nothing to drain
/// mid-capture; the poll inside the spin loop only watches for completion.
///
/// Every error here is fatal to the suite — each one means the loopback
/// plumbing itself is broken (or a frame hung), and some paths lose a
/// receiver with the failed transaction.
///
fn capture<'ch, const K: usize>(
    rx: [RxCh<'ch>; K],
    bufs: [&mut [PulseCode]; K],
    frames: &[(u8, &[u8])],
    armed: &[bool; K],
) -> Result<([RxCh<'ch>; K], [usize; K]), &'static str> {
    // The receivers go first so no frame's first edge can be missed; the
    // lines then idle low for far less than the idle threshold before TX
    // starts. A receiver whose transmitter sits this round out is not armed
    // at all: with no edges its idle counter never runs, so its transaction
    // would never complete.
    let mut txns = [const { None }; K];
    let mut parked = [const { None }; K];
    for (i, (ch, buf)) in rx.into_iter().zip(bufs).enumerate() {
        if !armed[i] {
            parked[i] = Some(ch);
            continue;
        }
        for code in buf.iter_mut() {
            code.reset();
        }
        match ch.receive(buf) {
            Ok(txn) => txns[i] = Some(txn),
            Err(_) => return Err("rx_receive"),
        }
    }

    let started = Instant::now();
    let mut timed_out = false;
    let send = DRIVER.send_blocking_all(frames, || {
        for txn in txns.iter_mut().flatten() {
            let _ = txn.poll();
        }
        if started.elapsed() > FRAME_TIMEOUT {
            timed_out = true;
            for &(ch, _) in frames {
                DRIVER.abort(ch);
            }
        }
    });
    if send.is_err() {
        return Err("tx_start");
    }
    if timed_out {
        return Err("tx_timeout");
    }

    // The frames are out; each receiver ends once its line has idled past the
    // threshold (375 us). Far more than that and something is replaying.
    let rx_deadline = Instant::now();
    loop {
        let mut all_done = true;
        for txn in txns.iter_mut().flatten() {
            if !txn.poll() {
                all_done = false;
            }
        }
        if all_done {
            break;
        }
        if rx_deadline.elapsed() > Duration::from_millis(50) {
            return Err("rx_no_idle");
        }
    }

    let mut totals = [0usize; K];
    let mut channels = parked;
    for (i, txn) in txns.into_iter().enumerate() {
        let Some(txn) = txn else {
            continue; // Not armed this round; the channel is already parked.
        };
        match txn.wait() {
            Ok((n, ch)) => {
                totals[i] = n;
                channels[i] = Some(ch);
            }
            Err(_) => return Err("rx_error"),
        }
    }
    // Every slot was either parked at arm time or just filled by `wait`.
    Ok((channels.map(|c| c.unwrap()), totals))
}

/// Print `E5: FAIL` with `reason` forever — for failures that break the
/// harness itself rather than one assertion.
fn fatal(reason: &'static str) -> ! {
    loop {
        esp_println::println!("E5: FAIL loopback_esp32 reason={reason}");
        let park = Instant::now();
        while park.elapsed() < Duration::from_millis(2000) {}
    }
}

/// Track overall verdict; keep running so one miss still yields a full
/// report.
struct Verdict {
    ok: bool,
    first_fail: &'static str,
}

impl Verdict {
    fn fail(&mut self, name: &'static str) {
        if self.ok {
            self.ok = false;
            self.first_fail = name;
        }
    }
}

/// Frame slices for one round, as `send_blocking_all` wants them.
fn starts<'a>(
    frames: &'a [[u8; MAX_FRAME_BYTES]; CHANNELS],
    lens: &[usize; CHANNELS],
) -> [(u8, &'a [u8]); CHANNELS] {
    [
        (0, &frames[0][..lens[0]]),
        (1, &frames[1][..lens[1]]),
        (2, &frames[2][..lens[2]]),
        (3, &frames[3][..lens[3]]),
    ]
}

/// The TX channel configuration both phases use.
fn tx_config() -> TxChannelConfig {
    TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output(true)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_memsize(BLOCKS_PER_CHANNEL)
}

/// The RX channel configuration both phases use, sized per receiver.
fn rx_config(blocks: u8) -> RxChannelConfig {
    RxChannelConfig::default()
        .with_clk_divider(1)
        .with_carrier_modulation(false)
        .with_filter_threshold(RX_FILTER_TICKS)
        .with_idle_threshold(IDLE_THRESHOLD_TICKS)
        .with_memsize(blocks)
}

pub fn run(peripherals: Peripherals) -> ! {
    esp_println::println!(
        "led-lab-esp32: P5 loopback self-test, GPIO16-19 TX ch0-3 -> RX ch4-7, no wires"
    );

    let mut verdict = Verdict {
        ok: true,
        first_fail: "",
    };

    // Phase A: four TX + four RX, one block each — known-answer decode,
    // per-bit timing, latch, cross-talk, golden dump, 100-frame soak. All
    // peripheral handles die when it returns.
    main_phase(peripherals, &mut verdict);

    // Phase B: the truncation test needs a 128-item capture on the victim,
    // and a wrap-less receiver can only grow by taking a neighbour's memory
    // block — so the whole peripheral is reconfigured: RX ch4 gets 2 blocks
    // (absorbing ch5's) and watches the victim, RX ch6/ch7 keep one block
    // each and watch two bystanders, TX ch3 runs unobserved (its driver
    // counters still assert isolation).
    //
    // SAFETY: `main_phase` consumed the only `Peripherals` instance and
    // returned, dropping every channel, pin and `Rmt` handle it created —
    // nothing else borrows any peripheral when this steal runs, and the
    // stolen set strictly replaces the dropped one.
    let stolen = unsafe { Peripherals::steal() };
    truncation_phase(stolen, &mut verdict);

    // --- Verdict, repeated so any capture window catches it -----------------
    let frames_done = DRIVER.stats(0).frames;
    loop {
        if verdict.ok {
            esp_println::println!(
                "E5: PASS loopback_esp32 channels={CHANNELS} frames={frames_done}"
            );
        } else {
            esp_println::println!(
                "E5: FAIL loopback_esp32 first_fail={} frames={frames_done}",
                verdict.first_fail
            );
        }
        let park = Instant::now();
        while park.elapsed() < Duration::from_millis(2000) {}
    }
}

/// Phase A: everything except truncation, on the symmetric 4 TX + 4 RX
/// configuration.
fn main_phase(peripherals: Peripherals, verdict: &mut Verdict) {
    let mut rmt = match Rmt::new(peripherals.RMT, RMT_CLOCK) {
        Ok(rmt) => rmt,
        Err(_) => fatal("rmt_init"),
    };
    install_isr(&mut rmt);

    // Routing option 1: split each pad into a frozen input/output signal pair
    // so the same pin feeds both RMT ends through the GPIO matrix.
    let (rx_sig0, tx_sig0) = Flex::new(peripherals.GPIO16).split();
    let (rx_sig1, tx_sig1) = Flex::new(peripherals.GPIO17).split();
    let (rx_sig2, tx_sig2) = Flex::new(peripherals.GPIO18).split();
    let (rx_sig3, tx_sig3) = Flex::new(peripherals.GPIO19).split();

    let config = tx_config();
    // Kept alive for the whole phase: dropping one would release that
    // channel's memory block and disconnect its pin.
    let tx_channels = match (
        rmt.channel0.configure_tx(&config),
        rmt.channel1.configure_tx(&config),
        rmt.channel2.configure_tx(&config),
        rmt.channel3.configure_tx(&config),
    ) {
        (Ok(c0), Ok(c1), Ok(c2), Ok(c3)) => [
            c0.with_pin(tx_sig0),
            c1.with_pin(tx_sig1),
            c2.with_pin(tx_sig2),
            c3.with_pin(tx_sig3),
        ],
        _ => fatal("tx_configure"),
    };

    let rxc = rx_config(RX_BLOCKS);
    let rx = match (
        rmt.channel4.configure_rx(&rxc),
        rmt.channel5.configure_rx(&rxc),
        rmt.channel6.configure_rx(&rxc),
        rmt.channel7.configure_rx(&rxc),
    ) {
        (Ok(c4), Ok(c5), Ok(c6), Ok(c7)) => [
            c4.with_pin(rx_sig0),
            c5.with_pin(rx_sig1),
            c6.with_pin(rx_sig2),
            c7.with_pin(rx_sig3),
        ],
        _ => fatal("rx_configure"),
    };

    esp32_rmt::init_tx();
    esp32_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

    let timings = channel_timings();
    for (ch, timing) in timings.iter().enumerate() {
        if DRIVER.configure_default_clock(ch as u8, timing).is_err() {
            fatal("configure");
        }
    }
    let nom: [Nominal; CHANNELS] = [
        Nominal::from_timing(&timings[0]),
        Nominal::from_timing(&timings[1]),
        Nominal::from_timing(&timings[2]),
        Nominal::from_timing(&timings[3]),
    ];

    esp_println::println!(
        "E5: MEASURE routing option=1_flex_split gpios=16,17,18,19 tx_ch=0-3 rx_ch=4-7 \
         tx_blocks={} rx_blocks={} rx_items_per_channel={} rx_wrap=0 \
         idle_threshold_ticks={} filter_ticks={} tol_ticks={}",
        BLOCKS_PER_CHANNEL,
        RX_BLOCKS,
        RX_CODES,
        IDLE_THRESHOLD_TICKS,
        RX_FILTER_TICKS,
        TOL_TICKS,
    );
    for (ch, timing) in timings.iter().enumerate() {
        esp_println::println!(
            "E5: MEASURE channel ch={} rx_ch={} leds={} t0h_ns={} t1h_ns={} latch_us={} \
             color_order={:?}",
            ch,
            ch + 4,
            TEST_LEDS[ch],
            timing.t0h_ns,
            timing.t1h_ns,
            timing.latch_us,
            timing.color_order,
        );
    }

    let mut bufs = [[PulseCode::end_marker(); RX_CODES]; CHANNELS];
    let mut bits = Bits::new();
    let mut frames = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut lens = [0usize; CHANNELS];
    // Per-channel wire-order expectations and decodes, kept for the
    // cross-talk comparison after every channel has been decoded.
    let mut expected = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut expected_len = [0usize; CHANNELS];
    let mut decoded = [[0u8; MAX_BITS / 8]; CHANNELS];
    let mut decoded_len = [0usize; CHANNELS];

    // --- Known-answer decode, per-bit timing, latch -------------------------
    // All four transmit every round; one receiver is armed per round, for the
    // reason spelled out at the soak below (two or more concurrent RMT RX
    // channels corrupt the transmitters' own output on this chip). The frames
    // are the fixed `TEST_FRAMES`, so round `ch` puts exactly the same
    // waveform on every wire that round 0 did — decoding one channel per round
    // costs nothing but the rounds.
    for ch in 0..CHANNELS {
        lens[ch] = TEST_LEDS[ch] * 3;
        frames[ch][..6].copy_from_slice(&TEST_FRAMES[ch]);
        expected_len[ch] = wire_bytes(
            &frames[ch][..lens[ch]],
            timings[ch].color_order,
            &mut expected[ch],
        );
    }

    let mut rx = rx;
    for ch in 0..CHANNELS {
        let armed: [bool; CHANNELS] = core::array::from_fn(|i| i == ch);
        let [b0, b1, b2, b3] = &mut bufs;
        let (rx_next, totals) = match capture(
            rx,
            [&mut b0[..], &mut b1[..], &mut b2[..], &mut b3[..]],
            &starts(&frames, &lens),
            &armed,
        ) {
            Ok(v) => v,
            Err(reason) => fatal(reason),
        };
        rx = rx_next;

        if let Err(reason) = parse(&bufs[ch][..totals[ch].min(RX_CODES)], &mut bits) {
            fatal(reason);
        }
        esp_println::println!(
            "E5: MEASURE capture ch={} items={} bits={} leading_low_ticks={} \
             trailing_low_ticks={} ended_high={}",
            ch,
            totals[ch],
            bits.len,
            bits.leading_low,
            bits.trailing_low,
            bits.ended_high as u8,
        );

        // Decode against this channel's own configuration.
        match decode(&bits, nom[ch].mid, &mut decoded[ch]) {
            Ok(n) => {
                decoded_len[ch] = n;
                if n == expected_len[ch] && decoded[ch][..n] == expected[ch][..n] {
                    esp_println::println!(
                        "E5: PASS loopback_decode ch={ch} bytes={n} bits={}",
                        bits.len
                    );
                } else {
                    verdict.fail("decode");
                    esp_println::print!("E5: FAIL loopback_decode ch={ch} bytes={n} got=");
                    for b in &decoded[ch][..n] {
                        esp_println::print!("{b:02X}");
                    }
                    esp_println::print!(" want=");
                    for b in &expected[ch][..expected_len[ch]] {
                        esp_println::print!("{b:02X}");
                    }
                    esp_println::println!();
                }
            }
            Err(reason) => {
                verdict.fail("decode");
                esp_println::println!(
                    "E5: FAIL loopback_decode ch={ch} reason={reason} bits={}",
                    bits.len
                );
            }
        }

        // Per-bit timing against this channel's own pulse codes.
        let stats = timing_stats(&bits, &nom[ch]);
        esp_println::println!(
            "E5: MEASURE timing ch={} zeros={} ones={} t0h_ticks={}..{} t1h_ticks={}..{} \
             period_ticks={}..{} nominal_t0h={} nominal_t1h={} nominal_period={}",
            ch,
            stats.zeros,
            stats.ones,
            stats.t0h_min,
            stats.t0h_max,
            stats.t1h_min,
            stats.t1h_max,
            stats.period_min,
            stats.period_max,
            nom[ch].t0h,
            nom[ch].t1h,
            nom[ch].t0h as u32 + nom[ch].t0l as u32,
        );
        match stats.violation {
            None => esp_println::println!(
                "E5: PASS loopback_timing ch={ch} tol_ticks={TOL_TICKS} bits={}",
                bits.len
            ),
            Some(i) => {
                verdict.fail("timing");
                esp_println::println!(
                    "E5: FAIL loopback_timing ch={ch} first_bad_bit={i} high_ticks={} \
                     low_ticks={}",
                    bits.high[i],
                    bits.low[i],
                );
            }
        }

        // Trailing low bounds the latch from below; a capture that ended on a
        // marker with no recorded low means the receiver saw at least the
        // idle threshold of low, which itself exceeds the latch.
        let latch_seen = if bits.ended_high {
            IDLE_THRESHOLD_TICKS as u32
        } else {
            bits.trailing_low
        };
        if latch_seen >= nom[ch].latch {
            esp_println::println!(
                "E5: PASS loopback_latch ch={ch} trailing_low_ticks={latch_seen} latch_ticks={}",
                nom[ch].latch
            );
        } else {
            verdict.fail("latch");
            esp_println::println!(
                "E5: FAIL loopback_latch ch={ch} trailing_low_ticks={latch_seen} latch_ticks={}",
                nom[ch].latch
            );
        }

        // The golden vector is channel 0's capture, verbatim — the same
        // WS2812/GRB frame the S3 recorded, now on classic silicon with three
        // other channels transmitting alongside it.
        if ch == 0 {
            esp_println::println!(
                "E5: MEASURE golden_begin chip=esp32 config=ws2812_grb clock_hz=80000000 \
                 tick_ns=12.5 frame_rgb=A53C0F0180FF pairs={}",
                bits.len
            );
            let mut i = 0;
            while i < bits.len {
                esp_println::print!("E5: MEASURE golden_pairs i={i}");
                let end = (i + 12).min(bits.len);
                while i < end {
                    esp_println::print!(" H{} L{}", bits.high[i], bits.low[i]);
                    i += 1;
                }
                esp_println::println!();
            }
            esp_println::println!(
                "E5: MEASURE golden_end trailing_low_ticks={} idle_threshold_ticks={}",
                bits.trailing_low,
                IDLE_THRESHOLD_TICKS
            );
        }
    }

    // --- Cross-talk: no channel decoded to another channel's pattern --------
    // The four expectations must be pairwise distinct first, or the check
    // below would be vacuous.
    let mut distinct = true;
    let mut crossed = None;
    for a in 0..CHANNELS {
        for b in 0..CHANNELS {
            if a == b {
                continue;
            }
            if expected_len[a] == expected_len[b]
                && expected[a][..expected_len[a]] == expected[b][..expected_len[b]]
            {
                distinct = false;
            }
            if decoded_len[a] == expected_len[b]
                && decoded_len[a] > 0
                && decoded[a][..decoded_len[a]] == expected[b][..expected_len[b]]
            {
                crossed = Some((a, b));
            }
        }
    }
    if distinct && crossed.is_none() {
        esp_println::println!("E5: PASS loopback_cross_talk channels={CHANNELS}");
    } else {
        verdict.fail("cross_talk");
        match crossed {
            Some((a, b)) => esp_println::println!(
                "E5: FAIL loopback_cross_talk ch={a} decoded_as_ch={b} distinct={}",
                distinct as u8
            ),
            None => {
                esp_println::println!("E5: FAIL loopback_cross_talk reason=patterns_not_distinct")
            }
        }
    }

    // --- 100-frame concurrent soak ------------------------------------------
    let soak_before: [_; CHANNELS] = [
        DRIVER.stats(0),
        DRIVER.stats(1),
        DRIVER.stats(2),
        DRIVER.stats(3),
    ];
    let mut mismatches = [0usize; CHANNELS];
    // **One witness at a time.** All four channels transmit every round — the
    // concurrency under test is on the *transmit* side — but only the
    // witnessed channel's receiver is armed.
    //
    // Arming several receivers at once is not a neutral act on this chip: two
    // or more RMT RX channels capturing concurrently corrupt what the *
    // transmitters* put on the wire, which is what made this soak red. The
    // corruption was confirmed on the pad itself by a CPU `GPIO_IN` witness
    // that never touches the RMT receiver (see `src/diag.rs`, `x5`/`x9`), and
    // it vanishes at one armed receiver: 0 misses in 400 frames against 5-8 %
    // with four. So this is a measurement artifact of the harness, and one
    // receiver is the configuration that measures the transmitter rather than
    // the instrument. Coverage goes *up*, not down: each channel is now
    // witnessed for `SOAK_FRAMES` frames while all four transmit, so the suite
    // asserts 4 x SOAK_FRAMES channel-frames instead of SOAK_FRAMES.
    for witness in 0..CHANNELS {
        let armed: [bool; CHANNELS] = core::array::from_fn(|i| i == witness);
        for f in 0..SOAK_FRAMES {
            for ch in 0..CHANNELS {
                for j in 0..lens[ch] {
                    // A different sequence per channel and per frame, so a stale
                    // half or a swapped channel cannot alias into a plausible
                    // stream.
                    frames[ch][j] = ((f * 31 + j * 7 + ch * 97 + 3) % 251) as u8;
                }
                expected_len[ch] = wire_bytes(
                    &frames[ch][..lens[ch]],
                    timings[ch].color_order,
                    &mut expected[ch],
                );
            }
            let [b0, b1, b2, b3] = &mut bufs;
            let (rx_next, totals) = match capture(
                rx,
                [&mut b0[..], &mut b1[..], &mut b2[..], &mut b3[..]],
                &starts(&frames, &lens),
                &armed,
            ) {
                Ok(v) => v,
                Err(reason) => fatal(reason),
            };
            rx = rx_next;

            let ch = witness;
            let ok = match parse(&bufs[ch][..totals[ch].min(RX_CODES)], &mut bits) {
                Ok(()) => match decode(&bits, nom[ch].mid, &mut decoded[ch]) {
                    Ok(n) => n == expected_len[ch] && decoded[ch][..n] == expected[ch][..n],
                    Err(_) => false,
                },
                Err(_) => false,
            };
            if ok {
                continue;
            }
            mismatches[ch] += 1;
            // Diagnostic detail for the first few misses per channel: what the
            // receiver saw, item by item, so a capture-side artifact can be
            // told from a wire-side one.
            if mismatches[ch] <= 2 {
                esp_println::print!(
                    "E5: MEASURE soak_miss ch={ch} frame={f} items={} bits={} got=",
                    totals[ch],
                    bits.len,
                );
                let n = bits.len.div_ceil(8).min(decoded[ch].len());
                for b in &decoded[ch][..n] {
                    esp_println::print!("{b:02X}");
                }
                esp_println::print!(" want=");
                for b in &expected[ch][..expected_len[ch]] {
                    esp_println::print!("{b:02X}");
                }
                esp_println::print!(" raw=");
                for code in &bufs[ch][..totals[ch].min(RX_CODES).min(28)] {
                    esp_println::print!(
                        " {}:{}/{}:{}",
                        matches!(code.level1(), Level::High) as u8,
                        code.length1(),
                        matches!(code.level2(), Level::High) as u8,
                        code.length2(),
                    );
                }
                esp_println::println!();
            }
        }
    }

    let mut soak_ok = true;
    for ch in 0..CHANNELS {
        let after = DRIVER.stats(ch as u8);
        let trips = after.guard_trips - soak_before[ch].guard_trips;
        let errors = after.errors - soak_before[ch].errors;
        let skips = after.guard_skips - soak_before[ch].guard_skips;
        let lag_num = after.refill_lag_sum - soak_before[ch].refill_lag_sum;
        let lag_den = after.refill_lag_count - soak_before[ch].refill_lag_count;
        let (lag_int, lag_frac) = mean_lag_tenths(lag_num, lag_den);
        esp_println::println!(
            "E5: MEASURE soak ch={} witnessed_frames={} tx_frames={} mismatches={} \
             guard_trips={} guard_skips={} errors={} refill_lag_avg_words={}.{} refills={}",
            ch,
            SOAK_FRAMES,
            SOAK_FRAMES * CHANNELS,
            mismatches[ch],
            trips,
            skips,
            errors,
            lag_int,
            lag_frac,
            lag_den,
        );
        if mismatches[ch] != 0 || trips != 0 || errors != 0 {
            soak_ok = false;
        }
    }
    if soak_ok {
        esp_println::println!(
            "E5: PASS loopback_soak witnessed_frames={SOAK_FRAMES} channels={CHANNELS} \
             tx_concurrent={CHANNELS} rx_armed=1"
        );
    } else {
        verdict.fail("soak");
        esp_println::println!(
            "E5: FAIL loopback_soak witnessed_frames={SOAK_FRAMES} rx_armed=1"
        );
    }
    // Mask every RMT cause before the handles below go out of scope: dropping
    // the last channel takes the peripheral's clock away, and an interrupt
    // arriving after that point wedges the CPU (see `disable_all_interrupts`).
    esp32_rmt::disable_all_interrupts();
    drop(rx);
    drop(tx_channels);
}

/// Phase B: truncation on one channel while the other three run.
///
/// The victim gets a frame far longer than its RAM window with its *second*
/// threshold interrupt swallowed by the core's per-channel test hook. It must
/// walk into its guard and stop after exactly [`TRUNC_EXPECT_BITS`] bits,
/// still reporting complete; the other three share every interrupt entry with
/// it and must finish their frames untouched. The receiver layout is the
/// asymmetric one described in [`run`].
fn truncation_phase(peripherals: Peripherals, verdict: &mut Verdict) {
    let mut rmt = match Rmt::new(peripherals.RMT, RMT_CLOCK) {
        Ok(rmt) => rmt,
        Err(_) => fatal("rmt_init_trunc"),
    };
    // No-op (the handler survives from phase A: binding lives in the
    // interrupt controller, not the RMT register file) — kept for the case
    // where the phases are ever run independently.
    install_isr(&mut rmt);

    let (rx_sig0, tx_sig0) = Flex::new(peripherals.GPIO16).split();
    let (rx_sig1, tx_sig1) = Flex::new(peripherals.GPIO17).split();
    let (rx_sig2, tx_sig2) = Flex::new(peripherals.GPIO18).split();
    let (_rx_sig3, tx_sig3) = Flex::new(peripherals.GPIO19).split();

    let config = tx_config();
    let _tx_channels = match (
        rmt.channel0.configure_tx(&config),
        rmt.channel1.configure_tx(&config),
        rmt.channel2.configure_tx(&config),
        rmt.channel3.configure_tx(&config),
    ) {
        (Ok(c0), Ok(c1), Ok(c2), Ok(c3)) => [
            c0.with_pin(tx_sig0),
            c1.with_pin(tx_sig1),
            c2.with_pin(tx_sig2),
            c3.with_pin(tx_sig3),
        ],
        _ => fatal("tx_configure_trunc"),
    };

    // RX ch4 takes two memory blocks (its own and ch5's) to fit the 96-bit
    // truncated frame plus trailing marker; ch5 is therefore unusable and the
    // remaining receivers watch two of the three bystanders.
    let rx = match (
        rmt.channel4.configure_rx(&rx_config(2)),
        rmt.channel6.configure_rx(&rx_config(RX_BLOCKS)),
        rmt.channel7.configure_rx(&rx_config(RX_BLOCKS)),
    ) {
        // Victim's receiver first: `capture` index 0 is the victim below.
        (Ok(c4), Ok(c6), Ok(c7)) => [
            c4.with_pin(rx_sig2),
            c6.with_pin(rx_sig0),
            c7.with_pin(rx_sig1),
        ],
        _ => fatal("rx_configure_trunc"),
    };

    esp32_rmt::init_tx();
    esp32_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

    let timings = channel_timings();
    for (ch, timing) in timings.iter().enumerate() {
        if DRIVER.configure_default_clock(ch as u8, timing).is_err() {
            fatal("configure_trunc");
        }
    }
    let nom: [Nominal; CHANNELS] = [
        Nominal::from_timing(&timings[0]),
        Nominal::from_timing(&timings[1]),
        Nominal::from_timing(&timings[2]),
        Nominal::from_timing(&timings[3]),
    ];

    esp_println::println!(
        "E5: MEASURE truncation_config victim={VICTIM} victim_rx_ch=4 victim_rx_items={} \
         bystander_rx=ch6:tx0,ch7:tx1 unobserved_tx=3 expected_stop_bits={TRUNC_EXPECT_BITS}",
        TRUNC_RX_CODES,
    );

    let mut frames = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut lens = [0usize; CHANNELS];
    let mut expected = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut expected_len = [0usize; CHANNELS];
    for ch in 0..CHANNELS {
        if ch as u8 == VICTIM {
            lens[ch] = TRUNC_LEDS * 3;
            for j in 0..lens[ch] {
                frames[ch][j] = ((j * 37 + 11) % 251) as u8;
            }
        } else {
            lens[ch] = TEST_LEDS[ch] * 3;
            frames[ch][..6].copy_from_slice(&TEST_FRAMES[ch]);
        }
        expected_len[ch] = wire_bytes(
            &frames[ch][..lens[ch]],
            timings[ch].color_order,
            &mut expected[ch],
        );
    }

    let trunc_before: [_; CHANNELS] = [
        DRIVER.stats(0),
        DRIVER.stats(1),
        DRIVER.stats(2),
        DRIVER.stats(3),
    ];
    DRIVER.suppress_thresholds_on(VICTIM, 1, 1);

    let mut victim_buf = [PulseCode::end_marker(); TRUNC_RX_CODES];
    let mut by0_buf = [PulseCode::end_marker(); RX_CODES];
    let mut by1_buf = [PulseCode::end_marker(); RX_CODES];
    let (_rx, totals) = match capture(
        rx,
        [&mut victim_buf[..], &mut by0_buf[..], &mut by1_buf[..]],
        &starts(&frames, &lens),
        // All four transmit; the three receivers each watch a transmitting
        // channel (ch3 is the unobserved one), so all three are armed.
        &[true; 3],
    ) {
        Ok(v) => v,
        Err(reason) => fatal(reason),
    };

    let victim_bits_written = DRIVER
        .channel(VICTIM)
        .map(|c| c.bits_emitted())
        .unwrap_or(0);

    // The targeted re-raise check (S3 finding, P4): after a suppressed
    // threshold, does the hardware raise `tx_thr_event` again without the
    // transmitter crossing another boundary? If it does, the driver services
    // the replacement refill after the guard has already stopped the
    // transmitter, and the bit cursor runs one half (32 bits here) ahead of
    // the wire — `bits_written == expected_stop + 32`. If it does not,
    // `bits_written == expected_stop`. Either way the *wire* is asserted
    // below; this line only records which semantics the classic has.
    let reraise_delta = victim_bits_written as isize - TRUNC_EXPECT_BITS as isize;
    esp_println::println!(
        "E5: MEASURE thr_reraise bits_written={victim_bits_written} \
         expected_stop_bits={TRUNC_EXPECT_BITS} delta={reraise_delta} \
         reraises_like_s3={}",
        (reraise_delta > 0) as u8,
    );

    let mut bits = Bits::new();
    let mut decoded = [0u8; MAX_BITS / 8];
    let mut isolation_ok = true;

    // (driver channel, capture slot, capture length) for the three observed
    // channels; ch3 has no receiver and is asserted on counters alone.
    let observed: [(u8, &[PulseCode]); 3] = [
        (VICTIM, &victim_buf[..totals[0].min(TRUNC_RX_CODES)]),
        (0, &by0_buf[..totals[1].min(RX_CODES)]),
        (1, &by1_buf[..totals[2].min(RX_CODES)]),
    ];

    for (ch, codes) in observed {
        let chu = ch as usize;
        let after = DRIVER.stats(ch);
        let trips = after.guard_trips - trunc_before[chu].guard_trips;
        let parsed = parse(codes, &mut bits).is_ok();
        let rx_bits = if parsed { bits.len } else { 0 };
        let decoded_ok = parsed
            && match decode(&bits, nom[chu].mid, &mut decoded) {
                Ok(n) => n <= expected_len[chu] && decoded[..n] == expected[chu][..n],
                Err(_) => false,
            };

        if ch == VICTIM {
            let refills = after.refill_lag_count - trunc_before[chu].refill_lag_count;
            esp_println::println!(
                "E5: MEASURE truncation ch={ch} role=victim bits_rx={rx_bits} \
                 expected_stop_bits={TRUNC_EXPECT_BITS} total_bits={} prefix_ok={} \
                 guard_trips_delta={trips} bits_written={victim_bits_written} refills={refills}",
                lens[chu] * 8,
                decoded_ok as u8,
            );
            // The **wire** is the authority on where the transmitter stopped:
            // the receiver must see exactly TRUNC_EXPECT_BITS bits, a clean
            // prefix, and then idle — no stale-half replay. `bits_written` is
            // deliberately *not* asserted (see the re-raise MEASURE above).
            if trips != 1 || rx_bits != TRUNC_EXPECT_BITS || !decoded_ok {
                isolation_ok = false;
            }
        } else {
            esp_println::println!(
                "E5: MEASURE truncation ch={ch} role=bystander bits_rx={rx_bits} \
                 guard_trips_delta={trips} decoded_ok={}",
                decoded_ok as u8,
            );
            if trips != 0 || !decoded_ok || rx_bits != expected_len[chu] * 8 {
                isolation_ok = false;
            }
        }
    }

    // The unobserved bystander (ch3): its frame must have completed exactly
    // once with no trips and no errors — the counters are its wire.
    let after3 = DRIVER.stats(3);
    let frames3 = after3.frames - trunc_before[3].frames;
    let trips3 = after3.guard_trips - trunc_before[3].guard_trips;
    let errors3 = after3.errors - trunc_before[3].errors;
    esp_println::println!(
        "E5: MEASURE truncation ch=3 role=unobserved frames_delta={frames3} \
         guard_trips_delta={trips3} errors_delta={errors3}"
    );
    if frames3 != 1 || trips3 != 0 || errors3 != 0 {
        isolation_ok = false;
    }

    if isolation_ok {
        esp_println::println!(
            "E5: PASS loopback_truncation victim={VICTIM} guard_trips=1 \
             stopped_at_bit={TRUNC_EXPECT_BITS} bystanders_clean=1"
        );
    } else {
        verdict.fail("truncation");
        esp_println::println!(
            "E5: FAIL loopback_truncation victim={VICTIM} expected_stop_bits={TRUNC_EXPECT_BITS} \
             bits_written={victim_bits_written}"
        );
    }

    // As in `main_phase`: mask every cause before the channel handles below go
    // out of scope and take the peripheral's clock with them. Without this the
    // suite reaches its verdict and then wedges on the way out of this
    // function, printing nothing ever again.
    esp32_rmt::disable_all_interrupts();
}

/// Mean refill lag in words, as an integer part and one decimal digit — the
/// same integer-only formatting the demo uses.
fn mean_lag_tenths(sum: i32, count: i32) -> (i32, i32) {
    if count == 0 {
        return (0, 0);
    }
    let tenths = sum.saturating_mul(10) / count;
    (tenths / 10, (tenths % 10).abs())
}
