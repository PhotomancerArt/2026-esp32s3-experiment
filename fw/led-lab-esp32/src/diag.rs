//! `diag` — the discriminating experiments for the classic-ESP32 OPEN RED
//! finding (4-channel concurrent soak misses ~5-8 % of channel-frames).
//!
//! The soak harness in [`crate::loopback`] can only tell you *that* a capture
//! disagreed with the frame that was sent. It cannot tell you **where** the
//! disagreement was introduced, because every one of its witnesses is an RMT
//! RX channel sharing the same 512-word RAM as the transmitters. This module
//! adds instruments that do not share that RAM, plus a small matrix of
//! configurations that separate the candidate mechanisms:
//!
//! * **X1** baseline — reproduce the soak in this module's own runner, so
//!   every later number is comparable.
//! * **X2** receiver count — 4 TX with exactly *one* receiver armed. If the
//!   misses survive, four concurrent receivers are not the cause.
//! * **X3** transmitter count — one fixed witness, 1/2/3/4 transmitters. Does
//!   the miss rate scale with the number of concurrent *fetchers*?
//! * **X4** receiver remap — the same four wires watched by the *reversed*
//!   set of RX channels. If corruption follows the **wire** it is TX-side; if
//!   it follows the **RX channel index** it is receiver-side.
//! * **X5** CPU wire witness — the decisive one. A tight IRAM loop
//!   edge-timestamps the pad through `GPIO_IN` with `CCOUNT` while the RMT RX
//!   channel captures the same wire. `GPIO_IN` is not the RMT: it does not
//!   touch RMT RAM, the RX filter, or the receive state machine. Agreement
//!   between the two witnesses on a corrupt frame means the **pad really
//!   carried the wrong pulse**; disagreement means the receiver invented it.
//! * **X6** TX RAM readback — after each frame, every transmitter's 64-word
//!   window is scanned for words that are not in that channel's own codebook.
//!   Catches RAM *writes* landing in the wrong block (e.g. a receiver's write
//!   address decoding into a transmitter's block), which the previous
//!   stamped-window test could not see because it ran with no receivers armed.
//! * **X7** pad separation — the whole X1 baseline again on four physically
//!   spread GPIOs instead of four adjacent ones (H1: simultaneous-switching
//!   coupling).
//! * **X8** start stagger — channels started ~half a bit period apart instead
//!   of back to back, so their RAM accesses stop being phase-locked.
//!
//! Nothing here is a test: every line is a `D5: MEASURE`. The `test_loopback`
//! suite is untouched and still the assertion of record.

use esp_hal::gpio::{Flex, Level};
use esp_hal::peripherals::{Peripherals, GPIO};
use esp_hal::rmt::{
    Channel, PulseCode, Rmt, Rx, RxChannelConfig, RxChannelCreator, TxChannelConfig,
    TxChannelCreator,
};
use esp_hal::time::{Duration, Instant};
use esp_hal::Blocking;

use lp_ws281x::{ChannelTiming, ColorOrder, PulseCodes, PulseItem};

use crate::esp32_rmt::{self, BLOCK_WORDS, BLOCKS_PER_CHANNEL, RAM_BASE, TX_BLOCKS};
use crate::{install_isr, DRIVER, FRAME_TIMEOUT, RMT_CLOCK};

/// Channels under test.
const CHANNELS: usize = 4;

/// One RX memory block per receiver — the same allocation the loopback
/// harness uses, so the numbers are comparable.
const RX_BLOCKS: u8 = 1;

/// Capture buffer in RMT items. The classic has no RX wrap, so one window is
/// the hard ceiling.
const RX_CODES: usize = 64;

/// Upper bound on bits parsed out of one capture.
const MAX_BITS: usize = RX_CODES;

/// Longest frame any experiment sends, in bytes.
const MAX_FRAME_BYTES: usize = 6;

/// 375 µs — above the 300 µs latch, as in the loopback harness.
const IDLE_THRESHOLD_TICKS: u16 = 30_000;

/// The receiver's input filter. Not optional on this chip (see the loopback
/// module docs); left at the harness value so this module measures the same
/// configuration the finding was raised against.
const RX_FILTER_TICKS: u8 = 15;

/// Frames per experiment block.
const FRAMES: usize = 100;

/// CPU cycles per RMT tick: 240 MHz core, 12.5 ns tick (80 MHz).
const CYCLES_PER_TICK: u32 = 3;

/// Edges the CPU witness can record: a 48-bit frame is 96 edges, plus room for
/// the overrun the finding sometimes shows.
const MAX_EDGES: usize = 128;

/// A witnessed bit whose period falls outside this window did not have both
/// its edges seen by the CPU loop — almost always because an interrupt ran
/// between them. Every legal period is exactly 100 ticks and the loop's
/// quantisation is one sample (~14 ticks) at each end, so anything beyond this
/// is the instrument blinking, not the wire misbehaving. Used only in the
/// refill blocks, where the RMT interrupt cannot be masked.
const WITNESS_PERIOD_TICKS: core::ops::RangeInclusive<u32> = 72..=128;

/// Give up on the witness loop after ~400 µs of CPU cycles.
const WITNESS_TIMEOUT_CYCLES: u32 = 96_000;

/// Low run that ends a witnessed frame: 5 µs, far above the longest legal
/// in-frame low (76 ticks = 950 ns) and far below the 300 µs latch.
const WITNESS_IDLE_CYCLES: u32 = 1_200;

/// Per-channel wire configuration — identical to the loopback harness so the
/// baseline block reproduces the finding exactly.
fn channel_timings() -> [ChannelTiming; CHANNELS] {
    [
        ChannelTiming::WS2812,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Rgb),
        ChannelTiming::WS2811,
        ChannelTiming::WS2812.with_color_order(ColorOrder::Bgr),
    ]
}

/// LEDs per channel in the loopback soak, reproduced for the baseline block.
const SOAK_LEDS: [usize; CHANNELS] = [2, 1, 2, 1];

/// LEDs per channel where the experiment must run **without any refill**: a
/// 24-bit frame is 26 RAM words, well under the 32-word threshold, so the
/// interrupt handler is never entered mid-frame and the RMT interrupt can be
/// masked for the whole transmission (which the CPU witness needs).
const SHORT_LEDS: [usize; CHANNELS] = [1, 1, 1, 1];

type RxCh<'ch> = Channel<'ch, Blocking, Rx>;

/// Nominal per-bit tick values, decoded from the same [`PulseCodes`] the
/// driver transmits.
#[derive(Clone, Copy)]
struct Nominal {
    t0h: u16,
    t1h: u16,
    mid: u16,
    zero: u32,
    one: u32,
    latch: u32,
}

impl Nominal {
    fn from_timing(timing: &ChannelTiming) -> Self {
        let codes = PulseCodes::at_default_clock(timing).unwrap();
        let zero = PulseItem::decode(codes.zero).unwrap();
        let one = PulseItem::decode(codes.one).unwrap();
        Self {
            t0h: zero.first.ticks,
            t1h: one.first.ticks,
            mid: (zero.first.ticks + one.first.ticks) / 2,
            zero: codes.zero,
            one: codes.one,
            latch: codes.latch,
        }
    }

    /// The high time this channel emits for `bit`.
    fn high_for(&self, bit: bool) -> u16 {
        if bit {
            self.t1h
        } else {
            self.t0h
        }
    }
}

/// One capture folded into per-bit high/low tick pairs.
struct Bits {
    high: [u16; MAX_BITS],
    low: [u32; MAX_BITS],
    len: usize,
}

impl Bits {
    const fn new() -> Self {
        Self {
            high: [0; MAX_BITS],
            low: [0; MAX_BITS],
            len: 0,
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
            self.idx = self.codes.len();
            return None;
        }
        Some((matches!(level, Level::High), ticks))
    }
}

/// Fold captured items into bits — the same reduction the loopback harness
/// performs, minus the latch bookkeeping this module does not assert.
fn parse(codes: &[PulseCode], out: &mut Bits) -> Result<(), &'static str> {
    *out = Bits::new();
    let mut started = false;
    let mut in_high = false;
    let mut high_acc: u32 = 0;
    let mut low_acc: u32 = 0;

    let halves = Halves {
        codes,
        idx: 0,
        second: false,
    };
    for (level, ticks) in halves {
        let ticks = ticks as u32;
        if !started {
            if !level {
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
        out.low[out.len] = if in_high { 0 } else { low_acc };
        out.len += 1;
    }
    Ok(())
}

/// The wire byte order for `frame` under `order`.
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

/// Bit `i` (MSB-first) of a wire-ordered byte string.
fn wire_bit(bytes: &[u8], i: usize) -> Option<bool> {
    let byte = *bytes.get(i / 8)?;
    Some(byte & (0x80 >> (i % 8)) != 0)
}

/// The soak's frame generator, verbatim, so the data stream is the one the
/// finding was raised against.
fn fill_frame(frame: &mut [u8], f: usize, ch: usize) {
    for (j, b) in frame.iter_mut().enumerate() {
        *b = ((f * 31 + j * 7 + ch * 97 + 3) % 251) as u8;
    }
}

/// Read the Xtensa cycle counter (`CCOUNT`), the finest clock this core has:
/// one count per CPU cycle, three per RMT tick at 240 MHz.
///
/// `esp_hal` re-exports `xtensa_lx`, so this needs no inline assembly (which
/// is still unstable on this architecture) and no `unsafe`.
#[inline(always)]
fn ccount() -> u32 {
    esp_hal::xtensa_lx::timer::get_cycle_count()
}

/// What ended a [`witness_edges`] loop, and how long it took — without these
/// a "clean" or "empty" CPU witness cannot be told from a blind one.
struct WitnessRun {
    edges: usize,
    /// `"idle"` (frame over), `"timeout"`, or `"full"` (buffer exhausted).
    reason: &'static str,
    /// CPU cycles the loop ran for.
    cycles: u32,
    /// Loop iterations — `cycles / iters` is the sampling period, and thus the
    /// resolution of every width this witness reports.
    iters: u32,
    /// Level on the pad at the loop's first sample.
    ///
    /// `start_frame` costs ~15 µs (it fills 26 RAM words), but the gap between
    /// its final `tx_start` write and this loop's first sample is only a few
    /// hundred nanoseconds — and the first high is 300-400 ns. So the loop
    /// routinely opens *inside* bit 0's high, and `out[0]` is then a **falling**
    /// edge rather than a rising one. Recording which it was is what lets
    /// [`witness_bits`] stay aligned instead of pairing every low with the
    /// wrong high.
    started_high: bool,
}

/// Edge-timestamp one GPIO from the CPU, in IRAM.
///
/// `out[0]` is the first **rising** edge (the line idles low between frames),
/// `out[1]` the falling edge that ends bit 0's high, and so on — so bit `k`'s
/// high time is `out[2k+1] - out[2k]` cycles.
///
/// Reading `GPIO_IN` is the only instrument in this firmware that observes the
/// pad without going through the RMT receiver: no RMT RAM, no input filter, no
/// receive state machine. The loop is `#[ram]` so a flash-cache miss cannot
/// blind it mid-frame.
#[esp_hal::ram]
fn witness_edges(mask: u32, out: &mut [u32; MAX_EDGES]) -> WitnessRun {
    let gpio = GPIO::regs();
    let start = ccount();
    let mut level = gpio.in_().read().bits() & mask != 0;
    let mut last_change = start;
    let mut n = 0usize;
    let mut iters = 0u32;
    let started_high = level;

    loop {
        iters = iters.wrapping_add(1);
        let now = ccount();
        let next = gpio.in_().read().bits() & mask != 0;
        if next != level {
            level = next;
            last_change = now;
            if n < MAX_EDGES {
                out[n] = now;
                n += 1;
            } else {
                return WitnessRun {
                    edges: n,
                    reason: "full",
                    cycles: now.wrapping_sub(start),
                    iters,
                    started_high,
                };
            }
        } else if n > 0 && !level && now.wrapping_sub(last_change) > WITNESS_IDLE_CYCLES {
            return WitnessRun {
                edges: n,
                reason: "idle",
                cycles: now.wrapping_sub(start),
                iters,
                started_high,
            };
        }
        if now.wrapping_sub(start) > WITNESS_TIMEOUT_CYCLES {
            return WitnessRun {
                edges: n,
                reason: "timeout",
                cycles: now.wrapping_sub(start),
                iters,
                started_high,
            };
        }
    }
}

/// Fold CPU edge timestamps into the same per-bit shape [`parse`] produces,
/// keeping only whole bits.
///
/// When the loop opened while the pad was already high, `edges[0]` is a
/// **falling** edge; that partial high is dropped so every pair fused below is
/// a genuine (high, low). Which wire bit `out[0]` is, is decided by the caller
/// from the frame's end — see `witness_block`.
fn witness_bits(edges: &[u32], started_high: bool, out: &mut Bits) {
    *out = Bits::new();
    let mut i = if started_high { 1usize } else { 0usize };
    while i + 1 < edges.len() && out.len < MAX_BITS {
        let high = edges[i + 1].wrapping_sub(edges[i]) / CYCLES_PER_TICK;
        let low = if i + 2 < edges.len() {
            edges[i + 2].wrapping_sub(edges[i + 1]) / CYCLES_PER_TICK
        } else {
            0
        };
        out.high[out.len] = high.min(u16::MAX as u32) as u16;
        out.low[out.len] = low;
        out.len += 1;
        i += 2;
    }
}

/// Compare one witness against the frame that was sent.
///
/// Returns `Ok(())` when every bit classifies to the expected value and the
/// bit count matches, otherwise the index of the first disagreement.
fn check(bits: &Bits, expect: &[u8], nom: &Nominal) -> Result<(), usize> {
    check_from(bits, expect, nom, 0)
}

/// Per-bit accounting for a witness that could not run with interrupts masked.
///
/// A refill runs the handler in the middle of the frame, and the CPU loop is
/// not sampling while it does — so *some* bits have a gap in them. Those show
/// up as a period far from the nominal 100 ticks and are skipped; every other
/// bit is still a valid observation of the pad. Returns
/// `(bits_checked, bits_wrong)`.
///
/// This is only sound while **no edge was lost** (the caller checks the bit
/// count): a lost edge pair would shift every later bit and turn one blind
/// spot into a frame full of false mismatches.
fn check_usable(bits: &Bits, expect: &[u8], nom: &Nominal, offset: usize) -> (usize, usize) {
    let mut checked = 0;
    let mut bad = 0;
    for i in 0..bits.len.saturating_sub(1) {
        if !WITNESS_PERIOD_TICKS.contains(&(bits.high[i] as u32 + bits.low[i])) {
            continue;
        }
        let Some(want) = wire_bit(expect, i + offset) else {
            continue;
        };
        checked += 1;
        if (bits.high[i] >= nom.mid) != (nom.high_for(want) >= nom.mid) {
            bad += 1;
        }
    }
    (checked, bad)
}

/// [`check`] restricted to wire bits `first..`, for comparing a full RMT
/// capture against the tail of a frame that the CPU witness only saw part of.
/// Both witnesses must be judged over the *same* bits or the contingency table
/// is comparing two different questions.
fn check_range(bits: &Bits, expect: &[u8], nom: &Nominal, first: usize) -> Result<(), usize> {
    let want_bits = expect.len() * 8;
    if bits.len != want_bits {
        return Err(want_bits.min(bits.len));
    }
    for i in first..bits.len {
        let Some(want) = wire_bit(expect, i) else {
            return Err(i);
        };
        if (bits.high[i] >= nom.mid) != (nom.high_for(want) >= nom.mid) {
            return Err(i);
        }
    }
    Ok(())
}

/// [`check`] for a witness whose first observed bit is wire bit `offset`.
fn check_from(bits: &Bits, expect: &[u8], nom: &Nominal, offset: usize) -> Result<(), usize> {
    let want_bits = expect.len() * 8;
    for i in 0..bits.len.max(want_bits.saturating_sub(offset)) {
        let want = match wire_bit(expect, i + offset) {
            Some(b) => nom.high_for(b),
            None => return Err(i + offset),
        };
        if i >= bits.len {
            return Err(i + offset);
        }
        if (bits.high[i] >= nom.mid) != (want >= nom.mid) {
            return Err(i + offset);
        }
    }
    if bits.len + offset != want_bits {
        return Err(want_bits.min(bits.len + offset));
    }
    Ok(())
}

/// Print, for the first disagreeing bit, what every other channel was emitting
/// at that same word index. This is the "another channel's word at the same
/// index" claim, made checkable.
#[allow(clippy::too_many_arguments)] // diagnostic dump needs every piece of context in one call
fn dump_miss(
    label: &str,
    tag: &str,
    wire: usize,
    frame: usize,
    bits: &Bits,
    at: usize,
    expect: &[[u8; MAX_FRAME_BYTES]; CHANNELS],
    expect_len: &[usize; CHANNELS],
    nom: &[Nominal; CHANNELS],
) {
    esp_println::print!(
        "D5: MEASURE miss label={label} src={tag} wire={wire} frame={frame} at={at} \
         rx_bits={} got_high={} got_low={} want_high={}",
        bits.len,
        bits.high.get(at).copied().unwrap_or(0),
        bits.low.get(at).copied().unwrap_or(0),
        wire_bit(&expect[wire][..expect_len[wire]], at)
            .map(|b| nom[wire].high_for(b))
            .unwrap_or(0),
    );
    esp_println::print!(" others_at_index=");
    for other in 0..CHANNELS {
        if other == wire {
            continue;
        }
        match wire_bit(&expect[other][..expect_len[other]], at) {
            Some(b) => esp_println::print!("ch{other}:{} ", nom[other].high_for(b)),
            None => esp_println::print!("ch{other}:- "),
        }
    }
    esp_println::println!();
}

/// The TX channel configuration every phase uses.
fn tx_config() -> TxChannelConfig {
    TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output(true)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_memsize(BLOCKS_PER_CHANNEL)
}

/// The RX channel configuration every phase uses.
fn rx_config() -> RxChannelConfig {
    RxChannelConfig::default()
        .with_clk_divider(1)
        .with_carrier_modulation(false)
        .with_filter_threshold(RX_FILTER_TICKS)
        .with_idle_threshold(IDLE_THRESHOLD_TICKS)
        .with_memsize(RX_BLOCKS)
}

/// Everything one experiment block varies.
#[derive(Clone, Copy)]
struct Block {
    label: &'static str,
    /// Which TX channels transmit this block.
    tx_on: [bool; CHANNELS],
    /// Which RX slots are armed. Slot `i` watches the wire named by the
    /// phase's `wire_of` map; a slot whose wire is silent must stay unarmed
    /// (its idle counter never runs, so its transaction would never end).
    arm: [bool; CHANNELS],
    leds: [usize; CHANNELS],
    /// Busy-wait between consecutive `start_frame` calls, in RMT ticks.
    stagger_ticks: u32,
    /// Scan the transmitters' RAM windows after every frame.
    ram_check: bool,
    frames: usize,
}

impl Block {
    const fn base(label: &'static str) -> Self {
        Self {
            label,
            tx_on: [true; CHANNELS],
            arm: [true; CHANNELS],
            leds: SOAK_LEDS,
            stagger_ticks: 0,
            ram_check: false,
            frames: FRAMES,
        }
    }
}

/// Busy-wait `ticks` RMT ticks using the cycle counter.
#[inline(always)]
fn spin_ticks(ticks: u32) {
    let until = ccount().wrapping_add(ticks * CYCLES_PER_TICK);
    while ccount().wrapping_sub(until) > u32::MAX / 2 {}
}

/// Words that legitimately appear in channel `ch`'s RAM window.
fn ram_word_ok(word: u32, nom: &Nominal) -> bool {
    word == 0 || word == nom.zero || word == nom.one || word == nom.latch
}

/// Scan every transmitter's RAM window for words outside its own codebook.
///
/// Returns `(foreign_words, first_ch, first_idx, first_word)`. A hit proves a
/// *write* landed in the wrong block — the one mechanism the earlier
/// stamped-window test could not see, because that test ran with no receivers
/// armed and therefore with no RX writes in flight.
fn scan_ram(tx_on: &[bool; CHANNELS], nom: &[Nominal; CHANNELS]) -> (usize, usize, usize, u32) {
    let mut hits = 0;
    let mut first = (0usize, 0usize, 0u32);
    for ch in 0..CHANNELS {
        if !tx_on[ch] {
            continue;
        }
        for idx in 0..BLOCK_WORDS {
            // SAFETY: `RAM_BASE` is the RMT peripheral's 512-word memory
            // window; `ch * BLOCK_WORDS + idx` is bounded by
            // `CHANNELS * BLOCK_WORDS` = 256 words, well inside it. The read is
            // volatile because the peripheral writes this memory.
            let word =
                unsafe { (RAM_BASE as *const u32).add(ch * BLOCK_WORDS + idx).read_volatile() };
            if !ram_word_ok(word, &nom[ch]) {
                if hits == 0 {
                    first = (ch, idx, word);
                }
                hits += 1;
            }
        }
    }
    (hits, first.0, first.1, first.2)
}

/// Start every enabled channel, wait for all of them, and poll the receivers
/// in between. Returns `Err` only when the plumbing itself broke.
fn send_round(
    tx_on: &[bool; CHANNELS],
    frames: &[[u8; MAX_FRAME_BYTES]; CHANNELS],
    lens: &[usize; CHANNELS],
    stagger_ticks: u32,
    mut spin: impl FnMut(),
) -> Result<(), &'static str> {
    for ch in 0..CHANNELS {
        if !tx_on[ch] {
            continue;
        }
        // SAFETY: `frames` is borrowed for this whole call and every channel
        // is driven to completion (or aborted) before it returns, so the bytes
        // the handler reads through its raw pointer stay alive, in place and
        // unmodified for the entire transmission.
        if unsafe { DRIVER.start_frame(ch as u8, &frames[ch][..lens[ch]]) }.is_err() {
            for c in 0..CHANNELS {
                DRIVER.abort(c as u8);
            }
            return Err("tx_start");
        }
        if stagger_ticks > 0 {
            spin_ticks(stagger_ticks);
        }
    }

    let started = Instant::now();
    while (0..CHANNELS).any(|ch| tx_on[ch] && !DRIVER.is_complete(ch as u8)) {
        spin();
        if started.elapsed() > FRAME_TIMEOUT {
            for ch in 0..CHANNELS {
                DRIVER.abort(ch as u8);
            }
            return Err("tx_timeout");
        }
    }
    Ok(())
}

/// Run one experiment block and print its result line.
///
/// `wire_of[i]` is the wire RX slot `i` is bound to (the binding itself is
/// made by the caller with [`esp_hal::rmt::RxChannelCreator::with_pin`], so a
/// remapped phase changes only this array and the pin it passed).
#[allow(clippy::too_many_arguments)] // one experiment configuration, not decomposable without a struct that would only ever have one caller
fn run_block(
    cfg: Block,
    rx: &mut [Option<RxCh<'_>>; CHANNELS],
    rx_ch_id: [u8; CHANNELS],
    wire_of: [usize; CHANNELS],
    timings: &[ChannelTiming; CHANNELS],
    nom: &[Nominal; CHANNELS],
    verbose: usize,
) {
    let mut frames = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut expect = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut expect_len = [0usize; CHANNELS];
    let mut lens = [0usize; CHANNELS];
    let mut bufs = [[PulseCode::end_marker(); RX_CODES]; CHANNELS];
    let mut bits = Bits::new();

    let mut misses = [0usize; CHANNELS];
    let mut dumped = [0usize; CHANNELS];
    let mut ram_hits = 0usize;
    let mut plumbing: Option<&'static str> = None;

    for f in 0..cfg.frames {
        for ch in 0..CHANNELS {
            lens[ch] = cfg.leds[ch] * 3;
            fill_frame(&mut frames[ch][..lens[ch]], f, ch);
            expect_len[ch] = wire_bytes(
                &frames[ch][..lens[ch]],
                timings[ch].color_order,
                &mut expect[ch],
            );
        }

        // Arm the receivers first: a frame's opening edge must not be missed.
        let mut txns = [const { None }; CHANNELS];
        let [b0, b1, b2, b3] = &mut bufs;
        let mut slot_bufs: [&mut [PulseCode]; CHANNELS] =
            [&mut b0[..], &mut b1[..], &mut b2[..], &mut b3[..]];
        for (slot, buf) in slot_bufs.iter_mut().enumerate() {
            if !cfg.arm[slot] {
                continue;
            }
            let Some(ch) = rx[slot].take() else { continue };
            for code in buf.iter_mut() {
                code.reset();
            }
            match ch.receive(&mut buf[..]) {
                Ok(txn) => txns[slot] = Some(txn),
                Err(_) => {
                    plumbing = Some("rx_receive");
                    break;
                }
            }
        }
        if plumbing.is_some() {
            break;
        }

        if let Err(reason) = send_round(&cfg.tx_on, &frames, &lens, cfg.stagger_ticks, || {
            for txn in txns.iter_mut().flatten() {
                let _ = txn.poll();
            }
        }) {
            plumbing = Some(reason);
        }

        let deadline = Instant::now();
        loop {
            let mut all = true;
            for txn in txns.iter_mut().flatten() {
                if !txn.poll() {
                    all = false;
                }
            }
            if all || plumbing.is_some() {
                break;
            }
            if deadline.elapsed() > Duration::from_millis(50) {
                plumbing = Some("rx_no_idle");
                break;
            }
        }

        let mut totals = [0usize; CHANNELS];
        for (slot, txn) in txns.into_iter().enumerate() {
            let Some(txn) = txn else { continue };
            match txn.wait() {
                Ok((n, ch)) => {
                    totals[slot] = n;
                    rx[slot] = Some(ch);
                }
                Err(_) => plumbing = Some("rx_error"),
            }
        }
        if plumbing.is_some() {
            break;
        }

        if cfg.ram_check {
            let (hits, ch, idx, word) = scan_ram(&cfg.tx_on, nom);
            if hits > 0 {
                if ram_hits == 0 {
                    esp_println::println!(
                        "D5: MEASURE ram_foreign label={} frame={f} words={hits} first_ch={ch} \
                         first_idx={idx} first_word={word:#010x}",
                        cfg.label,
                    );
                }
                ram_hits += hits;
            }
        }

        for slot in 0..CHANNELS {
            if !cfg.arm[slot] {
                continue;
            }
            let wire = wire_of[slot];
            let ok = match parse(&bufs[slot][..totals[slot].min(RX_CODES)], &mut bits) {
                Ok(()) => check(&bits, &expect[wire][..expect_len[wire]], &nom[wire]).is_ok(),
                Err(_) => false,
            };
            if ok {
                continue;
            }
            misses[wire] += 1;
            if dumped[wire] < verbose {
                dumped[wire] += 1;
                let at = check(&bits, &expect[wire][..expect_len[wire]], &nom[wire])
                    .err()
                    .unwrap_or(0);
                esp_println::println!(
                    "D5: MEASURE miss_ctx label={} wire={wire} rx_ch={} frame={f} items={}",
                    cfg.label,
                    rx_ch_id[slot],
                    totals[slot],
                );
                dump_miss(
                    cfg.label,
                    "rmt_rx",
                    wire,
                    f,
                    &bits,
                    at,
                    &expect,
                    &expect_len,
                    nom,
                );
            }
        }
    }

    let armed_wires = (0..CHANNELS).filter(|&s| cfg.arm[s]).count();
    let tx_count = cfg.tx_on.iter().filter(|&&on| on).count();
    esp_println::print!(
        "D5: MEASURE block label={} tx={tx_count} rx={armed_wires} stagger_ticks={} \
         frames={} ram_foreign={ram_hits} plumbing={} misses=",
        cfg.label,
        cfg.stagger_ticks,
        cfg.frames,
        plumbing.unwrap_or("ok"),
    );
    for (wire, n) in misses.iter().enumerate() {
        esp_println::print!("w{wire}:{n} ");
    }
    esp_println::println!();
}

/// Phase A/B/D share this shape: build the peripheral, bind four TX pins and
/// four RX channels through `wire_of`, then run `blocks`.
///
/// `pins` names the four pads in wire order; `wire_of[slot]` says which wire
/// RX slot `slot` (RMT channel `4 + slot`) watches.
macro_rules! phase {
    ($rmt_peripheral:expr, $wire_of:expr, $pins:expr, $body:expr) => {{
        let mut rmt = match Rmt::new($rmt_peripheral, RMT_CLOCK) {
            Ok(rmt) => rmt,
            Err(_) => {
                esp_println::println!("D5: MEASURE fatal reason=rmt_init");
                return;
            }
        };
        install_isr(&mut rmt);

        let (rx_sig, tx_sig) = $pins;
        let config = tx_config();
        let _tx = match (
            rmt.channel0.configure_tx(&config),
            rmt.channel1.configure_tx(&config),
            rmt.channel2.configure_tx(&config),
            rmt.channel3.configure_tx(&config),
        ) {
            (Ok(c0), Ok(c1), Ok(c2), Ok(c3)) => {
                let [s0, s1, s2, s3] = tx_sig;
                [
                    c0.with_pin(s0),
                    c1.with_pin(s1),
                    c2.with_pin(s2),
                    c3.with_pin(s3),
                ]
            }
            _ => {
                esp_println::println!("D5: MEASURE fatal reason=tx_configure");
                return;
            }
        };

        let rxc = rx_config();
        // Slot i is RMT channel 4+i and is bound to the pad of wire
        // `wire_of[i]` — the remap phase changes only which signal goes where.
        let wire_of: [usize; CHANNELS] = $wire_of;
        let mut sigs = rx_sig.map(Some);
        let mut take = |slot: usize| sigs[wire_of[slot]].take().unwrap();
        let mut rx = match (
            rmt.channel4.configure_rx(&rxc),
            rmt.channel5.configure_rx(&rxc),
            rmt.channel6.configure_rx(&rxc),
            rmt.channel7.configure_rx(&rxc),
        ) {
            (Ok(c4), Ok(c5), Ok(c6), Ok(c7)) => [
                Some(c4.with_pin(take(0))),
                Some(c5.with_pin(take(1))),
                Some(c6.with_pin(take(2))),
                Some(c7.with_pin(take(3))),
            ],
            _ => {
                esp_println::println!("D5: MEASURE fatal reason=rx_configure");
                return;
            }
        };

        esp32_rmt::init_tx();
        esp32_rmt::enable_tx_interrupts_for(&TX_BLOCKS);

        let timings = channel_timings();
        for (ch, timing) in timings.iter().enumerate() {
            if DRIVER.configure_default_clock(ch as u8, timing).is_err() {
                esp_println::println!("D5: MEASURE fatal reason=configure");
                return;
            }
        }
        let nom: [Nominal; CHANNELS] = [
            Nominal::from_timing(&timings[0]),
            Nominal::from_timing(&timings[1]),
            Nominal::from_timing(&timings[2]),
            Nominal::from_timing(&timings[3]),
        ];

        #[allow(clippy::redundant_closure_call)] // $body is a macro-parameter closure, not literal call syntax clippy can see through
        ($body)(&mut rx, wire_of, &timings, &nom);

        // Mask every cause before the handles drop: the last `Channel` takes
        // the RMT clock with it, and an interrupt after that point wedges the
        // CPU inside a max-priority handler (see `disable_all_interrupts`).
        esp32_rmt::disable_all_interrupts();
        drop(rx);
        drop(_tx);
    }};
}

pub fn run(peripherals: Peripherals) -> ! {
    esp_println::println!(
        "led-lab-esp32: D5 diagnostic — classic ESP32 concurrent-soak corruption, \
         cycles_per_tick={CYCLES_PER_TICK}"
    );

    phase_matrix(peripherals);

    // SAFETY: the phase above returned, dropping every peripheral handle it
    // created (channels, pins, `Rmt`); nothing borrows a peripheral when this
    // steal runs, and the stolen set strictly replaces the dropped one. The
    // same argument covers each later steal.
    phase_remap(unsafe { Peripherals::steal() });
    phase_witness(unsafe { Peripherals::steal() });
    phase_spread_pins(unsafe { Peripherals::steal() });

    loop {
        esp_println::println!("D5: MEASURE done");
        let park = Instant::now();
        while park.elapsed() < Duration::from_millis(2000) {}
    }
}

/// Phase A — the configuration matrix on the harness's own pins, RX slot `i`
/// watching wire `i` (RMT channel `4+i` ↔ TX channel `i`, as in the loopback
/// suite).
fn phase_matrix(peripherals: Peripherals) {
    let rmt_peripheral = peripherals.RMT;
    let pins = {
        let (r0, t0) = Flex::new(peripherals.GPIO16).split();
        let (r1, t1) = Flex::new(peripherals.GPIO17).split();
        let (r2, t2) = Flex::new(peripherals.GPIO18).split();
        let (r3, t3) = Flex::new(peripherals.GPIO19).split();
        ([r0, r1, r2, r3], [t0, t1, t2, t3])
    };
    esp_println::println!("D5: MEASURE phase name=matrix pins=16,17,18,19 rx_map=0,1,2,3");

    phase!(
        rmt_peripheral,
        [0, 1, 2, 3],
        pins,
        |rx: &mut [Option<RxCh<'_>>; CHANNELS], wire_of, timings: &_, nom: &_| {
            let ids = [4u8, 5, 6, 7];

            // X1 — the loopback soak, reproduced here.
            run_block(Block::base("x1_baseline"), rx, ids, wire_of, timings, nom, 2);

            // X6 — same, with the RAM scan on. Separated so the scan's own APB
            // traffic cannot be blamed for the baseline number.
            let mut x6 = Block::base("x6_ram_scan");
            x6.ram_check = true;
            run_block(x6, rx, ids, wire_of, timings, nom, 0);

            // X2 — four transmitters, exactly one receiver armed. If the
            // misses survive this, four concurrent receivers are not the cause.
            for solo in 0..CHANNELS {
                let mut b = Block::base(match solo {
                    0 => "x2_rx_only_w0",
                    1 => "x2_rx_only_w1",
                    2 => "x2_rx_only_w2",
                    _ => "x2_rx_only_w3",
                });
                b.arm = [false; CHANNELS];
                b.arm[solo] = true;
                run_block(b, rx, ids, wire_of, timings, nom, 1);
            }

            // X2b — every *pair* of armed receivers, four transmitters. If
            // the corruption is receivers contending with each other, this
            // says which pairs contend and how much: 4/5 and 6/7 share a
            // 128-word RAM half, 4/6 and 5/7 do not.
            const PAIRS: [(usize, usize, &str); 6] = [
                (0, 1, "x2b_rx45"),
                (0, 2, "x2b_rx46"),
                (0, 3, "x2b_rx47"),
                (1, 2, "x2b_rx56"),
                (1, 3, "x2b_rx57"),
                (2, 3, "x2b_rx67"),
            ];
            for (a, b, label) in PAIRS {
                let mut cfg = Block::base(label);
                cfg.arm = [false; CHANNELS];
                cfg.arm[a] = true;
                cfg.arm[b] = true;
                run_block(cfg, rx, ids, wire_of, timings, nom, 0);
            }

            // X2c — every *triple*, to see whether the rate simply tracks the
            // number of concurrent receivers.
            for out in 0..CHANNELS {
                let mut cfg = Block::base(match out {
                    0 => "x2c_no_rx4",
                    1 => "x2c_no_rx5",
                    2 => "x2c_no_rx6",
                    _ => "x2c_no_rx7",
                });
                cfg.arm = [true; CHANNELS];
                cfg.arm[out] = false;
                run_block(cfg, rx, ids, wire_of, timings, nom, 0);
            }

            // X3 — one fixed witness (wire 1, the worst channel in the
            // baseline), 1..4 concurrent transmitters.
            for n in 1..=CHANNELS {
                let mut b = Block::base(match n {
                    1 => "x3_tx1",
                    2 => "x3_tx2",
                    3 => "x3_tx3",
                    _ => "x3_tx4",
                });
                // Wire 1 must always transmit (it is the witnessed one); the
                // others join in index order.
                b.tx_on = [false; CHANNELS];
                b.tx_on[1] = true;
                for &other in [0usize, 2, 3].iter().take(n.saturating_sub(1)) {
                    b.tx_on[other] = true;
                }
                b.arm = [false, true, false, false];
                run_block(b, rx, ids, wire_of, timings, nom, 0);
            }

            // X8 — the same four channels started ~half a bit period apart, so
            // their RAM accesses stop being phase-locked.
            for &stagger in &[16u32, 50, 137] {
                let mut b = Block::base(match stagger {
                    16 => "x8_stagger16",
                    50 => "x8_stagger50",
                    _ => "x8_stagger137",
                });
                b.stagger_ticks = stagger;
                run_block(b, rx, ids, wire_of, timings, nom, 0);
            }

            // X1s — the short-frame variant every later phase uses (24 bits on
            // all four channels: no refill, so the interrupt can be masked).
            let mut short = Block::base("x1s_short_frames");
            short.leds = SHORT_LEDS;
            run_block(short, rx, ids, wire_of, timings, nom, 2);
        }
    );
}

/// Phase B — the same four wires, watched by the **reversed** set of RX
/// channels (RMT channel 4 now watches wire 3, 5 watches 2, …).
///
/// If the corruption follows the *wire* it is introduced before the receiver;
/// if it follows the *RX channel index* the receiver invented it.
fn phase_remap(peripherals: Peripherals) {
    let rmt_peripheral = peripherals.RMT;
    let pins = {
        let (r0, t0) = Flex::new(peripherals.GPIO16).split();
        let (r1, t1) = Flex::new(peripherals.GPIO17).split();
        let (r2, t2) = Flex::new(peripherals.GPIO18).split();
        let (r3, t3) = Flex::new(peripherals.GPIO19).split();
        ([r0, r1, r2, r3], [t0, t1, t2, t3])
    };
    esp_println::println!("D5: MEASURE phase name=remap pins=16,17,18,19 rx_map=3,2,1,0");

    phase!(
        rmt_peripheral,
        [3, 2, 1, 0],
        pins,
        |rx: &mut [Option<RxCh<'_>>; CHANNELS], wire_of, timings: &_, nom: &_| {
            let ids = [4u8, 5, 6, 7];
            run_block(Block::base("x4_remap"), rx, ids, wire_of, timings, nom, 2);
            let mut short = Block::base("x4_remap_short");
            short.leds = SHORT_LEDS;
            run_block(short, rx, ids, wire_of, timings, nom, 2);
        }
    );
}

/// Phase D — the baseline block again on four physically spread pads.
fn phase_spread_pins(peripherals: Peripherals) {
    let rmt_peripheral = peripherals.RMT;
    let pins = {
        let (r0, t0) = Flex::new(peripherals.GPIO16).split();
        let (r1, t1) = Flex::new(peripherals.GPIO22).split();
        let (r2, t2) = Flex::new(peripherals.GPIO25).split();
        let (r3, t3) = Flex::new(peripherals.GPIO27).split();
        ([r0, r1, r2, r3], [t0, t1, t2, t3])
    };
    esp_println::println!("D5: MEASURE phase name=spread_pins pins=16,22,25,27 rx_map=0,1,2,3");

    phase!(
        rmt_peripheral,
        [0, 1, 2, 3],
        pins,
        |rx: &mut [Option<RxCh<'_>>; CHANNELS], wire_of, timings: &_, nom: &_| {
            let ids = [4u8, 5, 6, 7];
            run_block(Block::base("x7_spread"), rx, ids, wire_of, timings, nom, 2);
            let mut short = Block::base("x7_spread_short");
            short.leds = SHORT_LEDS;
            run_block(short, rx, ids, wire_of, timings, nom, 0);
        }
    );
}

/// Phase C — **X5, the decisive experiment**: the CPU watches one pad through
/// `GPIO_IN` while that pad's RMT RX channel captures the same frame.
///
/// Both witnesses see the same 24-bit frame. Every channel sends one LED, so
/// no channel needs a refill and the RMT interrupt can be masked for the whole
/// transmission — the polling loop therefore cannot be blinded by the
/// handler, and the handler cannot be starved by the polling loop.
///
/// The four outcomes:
///
/// | CPU witness | RMT RX | conclusion |
/// |---|---|---|
/// | bad | bad | the **pad carried the wrong pulse** — real wire corruption |
/// | good | bad | the **receiver invented it** — harness artifact |
/// | bad | good | the receiver hid it (or the CPU loop missed an edge) |
/// | good | good | clean frame |
fn phase_witness(peripherals: Peripherals) {
    let rmt_peripheral = peripherals.RMT;
    let pin_gpio = [16u8, 17, 18, 19];
    let pins = {
        let (r0, t0) = Flex::new(peripherals.GPIO16).split();
        let (r1, t1) = Flex::new(peripherals.GPIO17).split();
        let (r2, t2) = Flex::new(peripherals.GPIO18).split();
        let (r3, t3) = Flex::new(peripherals.GPIO19).split();
        ([r0, r1, r2, r3], [t0, t1, t2, t3])
    };
    esp_println::println!(
        "D5: MEASURE phase name=cpu_witness pins=16,17,18,19 rx_map=0,1,2,3 leds=1,1,1,1"
    );

    phase!(
        rmt_peripheral,
        [0, 1, 2, 3],
        pins,
        |rx: &mut [Option<RxCh<'_>>; CHANNELS], _wire_of, timings: &[ChannelTiming; CHANNELS], nom: &[Nominal; CHANNELS]| {
            // Three arming levels, same instrument: four receivers (the
            // loopback harness's own configuration), one receiver, and none
            // at all — the last being the shape the deployment actually
            // ships, now with an oracle that does not need a receiver.
            for wire in 0..CHANNELS {
                let mut one = [false; CHANNELS];
                one[wire] = true;
                witness_block("x5_rx4", wire, pin_gpio[wire], rx, [true; CHANNELS], 1, timings, nom);
                witness_block("x9_rx1", wire, pin_gpio[wire], rx, one, 1, timings, nom);
                witness_block("x9_rx0", wire, pin_gpio[wire], rx, [false; CHANNELS], 1, timings, nom);
                // A refill block (frames long enough that the handler runs
                // mid-transmission) was tried here and **cannot work**: the
                // handler stops the CPU loop for longer than a bit, edges are
                // lost, and alignment — not just a width — is gone, so every
                // frame is discarded as blind. The refill case is covered
                // instead by the single-receiver RMT oracle, which the numbers
                // above show is a *non*-perturbing instrument: `x3_tx4` and
                // the `x2_rx_only_*` blocks run four transmitters with refills
                // on two of them and report zero misses over 400 frames.
            }
        }
    );
}


/// One CPU-witness block: `FRAMES` frames on all four channels, with `wire`
/// watched simultaneously by the CPU (through `GPIO_IN`) and by its own RMT RX
/// channel — **and with all four receivers armed**, which X2 showed is the
/// condition the corruption needs.
///
/// The witnessed channel is started **last** so the CPU loop is polling within
/// a microsecond or two of its first edge; `start_frame` costs several
/// microseconds per channel (it fills 26 RAM words), which is enough to miss a
/// whole 30 µs frame if the witnessed channel goes first.
#[allow(clippy::too_many_arguments)] // diagnostic block needs every piece of the experiment's config in one call
fn witness_block(
    label: &str,
    wire: usize,
    gpio: u8,
    rx: &mut [Option<RxCh<'_>>; CHANNELS],
    arm: [bool; CHANNELS],
    // `leds`: one LED (24 bits, 26 RAM words) stays under the 32-word
    // threshold and needs no refill; two LEDs (48 bits, 50 words) crosses it,
    // so the handler runs mid-frame and the RMT interrupt has to stay
    // unmasked. That is the point of the refill blocks: a refill is a **CPU
    // write into RMT RAM while other channels are fetching from it**, which is
    // what the deployment does continuously.
    leds: usize,
    timings: &[ChannelTiming; CHANNELS],
    nom: &[Nominal; CHANNELS],
) {
    let mut frames = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut expect = [[0u8; MAX_FRAME_BYTES]; CHANNELS];
    let mut expect_len = [0usize; CHANNELS];
    let lens = [leds * 3; CHANNELS];
    // One LED fits under the threshold; anything longer needs the handler, so
    // the interrupt cannot be masked for the frame.
    let mask_ints = leds == 1;
    let mut bufs = [[PulseCode::end_marker(); RX_CODES]; CHANNELS];
    let mut edges = [0u32; MAX_EDGES];
    let mut rx_bits = Bits::new();
    let mut cpu_bits = Bits::new();

    // The 2x2 contingency table the whole experiment exists to fill in.
    let (mut both_bad, mut rx_only, mut cpu_only, mut clean) = (0usize, 0usize, 0usize, 0usize);
    let mut agree_at = 0usize;
    let mut dumped = 0usize;
    let mut cpu_t0h = (u16::MAX, 0u16);
    let mut cpu_t1h = (u16::MAX, 0u16);
    let mut blind = 0usize;
    let mut checked_bits = 0usize;
    let mut bad_bits = 0usize;
    let mut worst_sample = 0u32;
    let mut plumbing = "ok";

    // Start order: every channel except the witnessed one, then the witnessed
    // one. All four are still started back to back, so they overlap exactly as
    // in the soak.
    let mut order = [0u8; CHANNELS];
    for (slot, ch) in order
        .iter_mut()
        .zip((0..CHANNELS).filter(|&ch| ch != wire))
    {
        *slot = ch as u8;
    }
    order[CHANNELS - 1] = wire as u8;

    for f in 0..FRAMES {
        for ch in 0..CHANNELS {
            fill_frame(&mut frames[ch][..lens[ch]], f, ch);
            expect_len[ch] = wire_bytes(
                &frames[ch][..lens[ch]],
                timings[ch].color_order,
                &mut expect[ch],
            );
        }

        // How many receivers are armed is the independent variable here: the
        // CPU witness is the same instrument in every block, so the three
        // arming levels (0, 1, 4) are directly comparable.
        let mut txns = [const { None }; CHANNELS];
        let [b0, b1, b2, b3] = &mut bufs;
        let mut slot_bufs: [&mut [PulseCode]; CHANNELS] =
            [&mut b0[..], &mut b1[..], &mut b2[..], &mut b3[..]];
        for (slot, buf) in slot_bufs.iter_mut().enumerate() {
            if !arm[slot] {
                continue;
            }
            let Some(ch) = rx[slot].take() else {
                plumbing = "rx_missing";
                break;
            };
            for code in buf.iter_mut() {
                code.reset();
            }
            match ch.receive(&mut buf[..]) {
                Ok(txn) => txns[slot] = Some(txn),
                Err(_) => plumbing = "rx_receive",
            }
        }
        if plumbing != "ok" {
            break;
        }

        // Mask every RMT cause for the duration of the frame. A 24-bit frame
        // is 26 RAM words against a 32-word threshold, so no refill is due and
        // nothing is lost by not servicing interrupts; `tx_end` stays latched
        // in `INT_RAW` and is delivered the moment the mask comes off.
        if mask_ints {
            esp32_rmt::disable_all_interrupts();
        }
        let start_cost = ccount();
        let mut started = true;
        for &ch in &order {
            // SAFETY: `frames` outlives this call and is not rewritten until
            // every channel has completed below, so the handler's raw pointer
            // stays valid for the whole transmission.
            if unsafe { DRIVER.start_frame(ch, &frames[ch as usize][..lens[ch as usize]]) }.is_err()
            {
                started = false;
            }
        }
        let start_cost = ccount().wrapping_sub(start_cost);
        let run = witness_edges(1u32 << gpio, &mut edges);
        if mask_ints {
            esp32_rmt::enable_tx_interrupts_for(&TX_BLOCKS);
        }

        if !started {
            plumbing = "tx_start";
        }
        let sample = run.cycles / run.iters.max(1);
        worst_sample = worst_sample.max(sample);

        let deadline = Instant::now();
        while (0..CHANNELS).any(|ch| !DRIVER.is_complete(ch as u8)) {
            for txn in txns.iter_mut().flatten() {
                let _ = txn.poll();
            }
            if deadline.elapsed() > FRAME_TIMEOUT {
                for ch in 0..CHANNELS {
                    DRIVER.abort(ch as u8);
                }
                plumbing = "tx_timeout";
                break;
            }
        }
        let rx_deadline = Instant::now();
        loop {
            let mut all = true;
            for txn in txns.iter_mut().flatten() {
                if !txn.poll() {
                    all = false;
                }
            }
            if all {
                break;
            }
            if rx_deadline.elapsed() > Duration::from_millis(50) {
                plumbing = "rx_no_idle";
                break;
            }
        }
        let mut total = 0usize;
        for (slot, txn) in txns.into_iter().enumerate() {
            let Some(txn) = txn else { continue };
            match txn.wait() {
                Ok((n, ch)) => {
                    rx[slot] = Some(ch);
                    if slot == wire {
                        total = n;
                    }
                }
                Err(_) => plumbing = "rx_error",
            }
        }
        if plumbing != "ok" {
            break;
        }

        // With no receiver on this wire there is no RMT-side verdict; the
        // contingency table then degenerates to the CPU witness alone, which
        // is exactly the point of the zero-receiver block.
        witness_bits(&edges[..run.edges], run.started_high, &mut cpu_bits);
        let want_bits = expect_len[wire] * 8;

        // **Align the CPU witness on the end of the frame, not the start.**
        // The loop opens a few hundred nanoseconds to a few microseconds after
        // the transmitter does (how many depends on where the caller's code
        // landed in flash), so the first bit it sees is not always bit 0. It
        // always runs to the idle that ends the frame, though, and with the
        // RMT interrupt masked it cannot lose an edge in between — so the bits
        // it holds are unambiguously the *last* `cpu_bits.len` of the frame.
        let offset = want_bits.saturating_sub(cpu_bits.len);
        let cpu_ok = check_from(&cpu_bits, &expect[wire][..expect_len[wire]], &nom[wire], offset);

        // The RMT capture is judged over the same bit range, so "the CPU saw
        // it and the receiver did not" cannot be an artifact of the two
        // witnesses covering different parts of the frame.
        let rx_ok = if arm[wire] {
            match parse(&bufs[wire][..total.min(RX_CODES)], &mut rx_bits) {
                Ok(()) => check_range(&rx_bits, &expect[wire][..expect_len[wire]], &nom[wire], offset),
                Err(_) => Err(0),
            }
        } else {
            rx_bits = Bits::new();
            Ok(())
        };

        // A CPU witness that did not see a whole frame is not evidence about
        // anything; count those separately rather than letting them land in
        // the contingency table.
        // An interrupt between two edges makes the CPU loop miss them; those
        // frames are not evidence either way.
        let interrupted = (0..cpu_bits.len.saturating_sub(1)).any(|i| {
            !WITNESS_PERIOD_TICKS.contains(&(cpu_bits.high[i] as u32 + cpu_bits.low[i]))
        });
        // With interrupts masked a gap means the instrument failed and the
        // frame is thrown away; with them unmasked a gap is expected once per
        // frame, so the frame is kept and only the affected bits are skipped
        // (see `check_usable`). Either way a *lost edge* — a bit count that
        // does not add up — discards the frame, because alignment is gone.
        if run.reason != "idle"
            || cpu_bits.len == 0
            || cpu_bits.len > want_bits
            || (mask_ints && interrupted)
        {
            blind += 1;
            if blind <= 2 {
                esp_println::println!(
                    "D5: MEASURE witness_blind label={label} wire={wire} frame={f} \
                     interrupted={interrupted} reason={} edges={} \
                     cpu_bits={} offset={offset} rx_bits={} cycles={} iters={} \
                     sample_cycles={sample} start_cost_cycles={start_cost}",
                    run.reason,
                    run.edges,
                    cpu_bits.len,
                    rx_bits.len,
                    run.cycles,
                    run.iters,
                );
            }
            continue;
        }

        let (ck, bad) =
            check_usable(&cpu_bits, &expect[wire][..expect_len[wire]], &nom[wire], offset);
        checked_bits += ck;
        bad_bits += bad;
        // In refill mode the frame-level verdict would fire on every
        // interrupt-blinded bit, so the contingency table below is only
        // meaningful with interrupts masked; `cpu_bits_wrong` is the number
        // that carries the refill blocks' result.
        let cpu_ok = if mask_ints || !interrupted {
            cpu_ok
        } else if bad == 0 {
            Ok(())
        } else {
            Err(0)
        };

        // Track what the CPU loop's resolution actually is, so a "clean CPU
        // witness" can never be a blind one.
        for i in 0..cpu_bits.len {
            let h = cpu_bits.high[i];
            if h < nom[wire].mid {
                cpu_t0h = (cpu_t0h.0.min(h), cpu_t0h.1.max(h));
            } else {
                cpu_t1h = (cpu_t1h.0.min(h), cpu_t1h.1.max(h));
            }
        }

        match (cpu_ok.is_err(), rx_ok.is_err()) {
            (true, true) => {
                both_bad += 1;
                if cpu_ok.err() == rx_ok.err() {
                    agree_at += 1;
                }
            }
            (false, true) => rx_only += 1,
            (true, false) => cpu_only += 1,
            (false, false) => clean += 1,
        }

        if (cpu_ok.is_err() || rx_ok.is_err()) && dumped < 4 {
            dumped += 1;
            esp_println::println!(
                "D5: MEASURE witness_miss wire={wire} frame={f} rx_items={total} \
                 rx_bits={} cpu_edges={} cpu_bits={} cpu_offset={offset} rx_at={:?} \
                 cpu_at={:?} sample_cycles={sample}",
                rx_bits.len,
                run.edges,
                cpu_bits.len,
                rx_ok.err(),
                cpu_ok.err(),
            );
            if let Err(at) = rx_ok {
                dump_miss(
                    "x5", "rmt_rx", wire, f, &rx_bits, at, &expect, &expect_len, nom,
                );
            }
            // The CPU witness is not passed to `dump_miss`: that helper indexes
            // both the capture and the expectation by the same wire-bit index,
            // which only holds for a witness with `offset == 0`. The
            // `witness_raw` line below reports it in full instead.
            esp_println::print!(
                "D5: MEASURE witness_raw wire={wire} frame={f} cpu_first_bit={offset} cpu_high="
            );
            for i in 0..cpu_bits.len.min(28) {
                esp_println::print!("{} ", cpu_bits.high[i]);
            }
            esp_println::print!("| rx_high=");
            for i in 0..rx_bits.len.min(28) {
                esp_println::print!("{} ", rx_bits.high[i]);
            }
            esp_println::println!();
        }
    }

    esp_println::println!(
        "D5: MEASURE witness label={label} rx_armed={} wire={wire} gpio={gpio} \
         leds={leds} frames={FRAMES} usable={} clean={clean} \
         both_bad={both_bad} agree_at={agree_at} rx_only_bad={rx_only} cpu_only_bad={cpu_only} \
         blind={blind} cpu_bits_checked={checked_bits} cpu_bits_wrong={bad_bits} \
         worst_sample_cycles={worst_sample} cpu_t0h_ticks={}..{} \
         cpu_t1h_ticks={}..{} nominal_t0h={} nominal_t1h={} plumbing={plumbing}",
        arm.iter().filter(|&&a| a).count(),
        FRAMES - blind,
        if cpu_t0h.0 == u16::MAX { 0 } else { cpu_t0h.0 },
        cpu_t0h.1,
        if cpu_t1h.0 == u16::MAX { 0 } else { cpu_t1h.0 },
        cpu_t1h.1,
        nom[wire].t0h,
        nom[wire].t1h,
    );
}
