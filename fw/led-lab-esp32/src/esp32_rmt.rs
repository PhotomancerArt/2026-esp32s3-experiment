//! Classic ESP32 (LX6) RMT backend for [`lp_ws281x::RmtHw`].
//!
//! All the chip knowledge in this firmware lives here: the RMT RAM address,
//! the `CHnCONF1` start/stop dance, the interrupt-register bit layout, and the
//! fact that `MEM_RADDR_EX` is an offset into the *whole* RMT RAM rather than
//! into the channel's own window. Everything else — what to write and when —
//! is [`lp_ws281x::Ws281xDriver`], which never sees a register name.
//!
//! Register derivation sources (license-safe, in the order they settled each
//! question): esp-hal 1.1.1 `src/rmt.rs` `chip_specific` module for
//! `any(esp32, esp32s2)` (MIT/Apache-2.0), the `esp32` PAC 0.40.2 field docs,
//! and `esp-metadata-generated` 0.4.0 (`rmt.*` properties for `esp32`). No
//! GPL source was consulted.
//!
//! # Where the RAM is
//!
//! The classic ESP32 RMT RAM sits at **`RMT_BASE + 0x800` = `0x3FF5_6800`**
//! (`esp-metadata-generated` `rmt.ram_start` = `1073047552`; PAC RMT base =
//! `0x3FF5_6000`). That is the *same* `+0x800` offset as the ESP32-S3 —
//! coincidence, not shared layout: the classic has eight 64-word blocks
//! (512 words) against the S3's eight 48-word ones, and a smaller register
//! file in front of them. The C6's `+0x400` remains the odd one out.
//! [`probe_ram_address`] proves the constant on silicon at every demo boot.
//!
//! # Structural differences from the S3 backend (each verified in the sources
//! above, none recalled from memory)
//!
//! * **8 channels, each TX-or-RX.** There is no fixed TX/RX split; a channel
//!   transmits because `CHnCONF1.tx_start` was set and receives because
//!   `rx_en` was. This backend drives channels `0..TX_CHANNELS` as
//!   transmitters and never touches the rest.
//! * **64-word blocks** (`rmt.channel_ram_size` = 64) → 32-word halves. The
//!   bit-cursor core handles the odd 1⅓-LEDs-per-half split untouched.
//! * **Per-channel `CHnCONF0`/`CHnCONF1`** instead of the S3's single
//!   `CH_TX_CONF0`: divider/memsize/idle-threshold live in CONF0; start/reset
//!   /owner/idle-output bits in CONF1. There is **no `conf_update`** — writes
//!   take effect immediately (esp-hal's `update()` is a no-op here).
//! * **`INT_*` bits interleave by channel**: `chN_tx_end` = bit `3N`,
//!   `chN_rx_end` = `3N+1`, `chN_err` = `3N+2`, and `chN_tx_thr_event` = bit
//!   `24+N` (PAC `int_raw` field docs). The S3 groups by event
//!   (`tx_end` 0..=3, `tx_err` 4..=7, `tx_thr_event` 8..=11). Note `ch_err`
//!   is a *combined* TX/RX error bit — there is no separate `tx_err`. RX
//!   errors cannot reach [`RmtHw::take_interrupts`] because this firmware
//!   never sets `int_ena` for channels it does not transmit on, and the
//!   snapshot reads `INT_ST` (= raw & ena).
//! * **Wrap enable is global**: `APB_CONF.mem_tx_wrap_en` (bit 1), not a
//!   per-channel `mem_tx_wrap_en` in the channel's own conf register. Set
//!   once by [`init_tx`]; every transmitting channel here wants wrap.
//! * **No immediate TX stop.** `rmt.has_tx_immediate_stop` = false: there is
//!   no `tx_stop` bit anywhere on this chip. esp-hal stops a channel by
//!   filling its whole RAM window with end markers, and [`RmtHw::stop_tx`]
//!   here does the same — the transmitter halts at the next word boundary
//!   (≤ 1.25 µs for WS2812 timing) and raises `tx_end`.
//! * **`mem_owner` exists** (`CHnCONF1` bit 5, 1 = receiver owns the RAM):
//!   [`RmtHw::start_tx`] clears it for the channel and for every extra block
//!   the window extends into, exactly as esp-hal's classic `start_tx` does.
//! * `CHnSTATUS.mem_raddr_ex` is 10 bits (max 1023 — wide enough for all 512
//!   words, the tell that it is an absolute offset) and `CH_TX_LIM.tx_lim` is
//!   9 bits (max 511; a full 8-block window of 512 words would not fit, so an
//!   all-blocks-on-one-channel plan is rejected by the width mask — not a
//!   configuration this firmware uses).

use esp_hal::peripherals::RMT;
use lp_ws281x::{BlockPlan, InterruptFlags, RmtHw};

/// Channels this build transmits on. The classic ESP32 has eight RMT channels,
/// any of which can be a transmitter; the loopback/demo/stress builds use the
/// first four (keeping 4–7 free to be receivers), the 8-TX soak and the
/// channel-count sweep use all eight.
///
/// Note that the sweep varies its *active* channel count at run time between
/// 1 and this constant — the block plan (and therefore the 32-word refill
/// deadline) has to stay identical across its cells, so all eight blocks are
/// allocated whatever the cell uses.
#[cfg(any(feature = "soak_8tx", feature = "sweep_channels"))]
pub const TX_CHANNELS: usize = TOTAL_TX_BLOCKS;
#[cfg(not(any(feature = "soak_8tx", feature = "sweep_channels")))]
pub const TX_CHANNELS: usize = 4;

/// Words in one classic-ESP32 RMT memory block (`rmt.channel_ram_size`).
pub const BLOCK_WORDS: usize = 64;

/// Memory blocks the chip has to hand out to transmitters.
///
/// Only read by the `TX_CHANNELS` arm above, which itself is `cfg`'d to the
/// `soak_8tx`/`sweep_channels` builds — so the default (4-channel) build sees
/// it as unread.
#[allow(dead_code, reason = "used by the soak_8tx/sweep_channels TX_CHANNELS arm above, cfg'd out of the default build")]
pub const TOTAL_TX_BLOCKS: usize = 8;

/// Memory blocks given to each transmitting channel.
///
/// One block each (the default): a 64-word window halves into 32 words = 1⅓
/// LEDs, the tightest refill deadline this chip poses (~40 µs at 800 kHz) and
/// the only way to get all eight outputs. See [`lp_ws281x::BlockPlan`] for the
/// trade.
///
/// Raising it is the lever against the ISR-throughput ceiling: the classic
/// sustains only ~48 000 refills/s, and a continuously-transmitting channel
/// demands `800_000 / half_words` of them, so 32-word halves saturate at two
/// channels while 64-word halves (2 blocks) push that to roughly four. The
/// sweep divides its channel count accordingly so the plan always fits.
///
/// NOTE: `option_env!` is read at compile time and does **not** make cargo
/// rebuild when the variable changes — `touch` this file after changing it.
pub const BLOCKS_PER_CHANNEL: u8 = match option_env!("BLOCKS_PER_CHANNEL") {
    Some(s) => match u8::from_str_radix(s, 10) {
        Ok(v) if v > 0 => v,
        _ => 1,
    },
    None => 1,
};

/// The TX-side allocation of RMT memory blocks, validated at compile time.
/// The *same* value is handed to the driver and to [`Esp32Rmt`], so window
/// sizes can never disagree.
pub const TX_BLOCKS: BlockPlan<TX_CHANNELS> = match BlockPlan::uniform(BLOCKS_PER_CHANNEL) {
    Ok(plan) => plan,
    Err(_) => panic!("BLOCKS_PER_CHANNEL does not divide the classic ESP32's RMT memory blocks"),
};

/// RAM words a channel owns under [`TX_BLOCKS`], for the channels that have
/// any. Reported by the demo's `E1: MEASURE` line.
#[cfg_attr(not(demo_build), allow(dead_code))]
pub const CHANNEL_WORDS: usize = BLOCK_WORDS * BLOCKS_PER_CHANNEL as usize;

/// Total RAM claimed by the TX-side plan, in words — the bound every pointer
/// here respects. With `TX_CHANNELS = 4` this is blocks 0..=3; blocks 4..=7
/// belong to the loopback receivers and must never be written by this code.
const TX_RAM_WORDS: usize = BLOCK_WORDS * TX_CHANNELS;

/// Byte offset from the RMT peripheral base to the start of RMT RAM.
///
/// **`0x800` on the classic ESP32** — numerically the same as the S3's, per
/// the module docs. Verified on silicon by [`probe_ram_address`].
pub const RAM_OFFSET: usize = 0x800;

/// Absolute address of the classic ESP32 RMT RAM (`0x3FF5_6800`).
pub const RAM_BASE: usize = 0x3FF5_6000 + RAM_OFFSET;

/// Widest value `CH_TX_LIM.tx_lim` (bits 0..=8) can hold.
const TX_LIM_MAX: u16 = 0x1FF;

/// Bit position of `chN_tx_end` in the `INT_*` registers: three bits per
/// channel, interleaved by channel (see module docs).
const fn int_tx_end_bit(ch: u8) -> u32 {
    1u32 << (3 * ch)
}

/// Bit position of `chN_err` (combined TX/RX error) in the `INT_*` registers.
const fn int_err_bit(ch: u8) -> u32 {
    1u32 << (3 * ch + 2)
}

/// Bit position of `chN_tx_thr_event` in the `INT_*` registers.
const fn int_thr_bit(ch: u8) -> u32 {
    1u32 << (24 + ch)
}

/// The `INT_*` bits belonging to TX channel `ch` (end, err, thr_event).
/// `chN_rx_end` (bit `3N+1`) is deliberately excluded: it is an RX cause and
/// this backend never enables or clears causes for a role it does not drive.
const fn tx_event_mask(ch: u8) -> u32 {
    int_tx_end_bit(ch) | int_err_bit(ch) | int_thr_bit(ch)
}

/// Pointer to word `word_idx` of channel `ch`'s RAM window under `blocks`, or
/// `None` if either index is out of range.
///
/// The bounds check costs one compare per word on the refill path and buys a
/// handler that can never scribble outside the TX-side RAM (in the loopback
/// build, blocks 4..=7 are the receivers' — a stray write there would corrupt
/// a capture, not just a frame).
#[inline(always)]
fn ram_word(blocks: &BlockPlan<TX_CHANNELS>, ch: u8, word_idx: usize) -> Option<*mut u32> {
    if word_idx >= blocks.window_words(ch, BLOCK_WORDS) {
        // Covers an out-of-range or absorbed channel too: both have no window.
        return None;
    }
    let index = blocks.window_start(ch, BLOCK_WORDS) + word_idx;
    if index >= TX_RAM_WORDS {
        return None;
    }
    // SAFETY: `RAM_BASE` is the RMT peripheral's memory window, at least
    // `TX_RAM_WORDS` u32 words long (the peripheral has 512); `index` was just
    // bounded to that range, so the result stays inside one allocated MMIO
    // object and the byte offset (< 2 KiB) cannot overflow an `isize`.
    Some(unsafe { (RAM_BASE as *mut u32).add(index) })
}

/// The seven register operations `lp-ws281x` needs, on the classic ESP32.
///
/// Carries only the memory-block plan: everything else it addresses is
/// memory-mapped, so it is `const`-constructible and can live in a `static`
/// shared with the interrupt handler.
#[derive(Debug, Clone, Copy)]
pub struct Esp32Rmt {
    blocks: BlockPlan<TX_CHANNELS>,
}

impl Esp32Rmt {
    /// A backend handle for `blocks`. Touches no hardware.
    ///
    /// Pass the same plan to [`lp_ws281x::Ws281xDriver::with_blocks`].
    pub const fn new(blocks: BlockPlan<TX_CHANNELS>) -> Self {
        Self { blocks }
    }
}

impl Default for Esp32Rmt {
    fn default() -> Self {
        Self::new(TX_BLOCKS)
    }
}

impl RmtHw for Esp32Rmt {
    #[inline]
    fn ram_words(&self, ch: u8) -> usize {
        self.blocks.window_words(ch, BLOCK_WORDS)
    }

    #[inline]
    fn write_ram(&self, ch: u8, word_idx: usize, value: u32) {
        let Some(ptr) = ram_word(&self.blocks, ch, word_idx) else {
            return;
        };
        // SAFETY: `ram_word` returned an in-range, naturally aligned pointer
        // into the RMT RAM window. The write must be volatile: the transmitter
        // reads this memory behind the compiler's back, so the store can be
        // neither elided nor reordered with the surrounding register writes.
        unsafe { ptr.write_volatile(value) };
    }

    #[inline]
    fn set_tx_threshold(&self, ch: u8, words: u16) {
        if ch as usize >= TX_CHANNELS {
            return;
        }
        // **The classic's `tx_lim` is a repeating count, not a position.**
        //
        // The PAC puts it plainly: "when channel N sends more than
        // `tx_lim` datas then channel N produces the relative interrupt" — a
        // count of *entries sent*, which re-arms itself, so a fixed value
        // fires once per that many words for the whole frame. esp-hal's own
        // classic driver relies on exactly that: it programs
        // `memsize.codes() / 2` once at the start of a transmission and never
        // touches the register again.
        //
        // The driver core, written against the S3 where the threshold names a
        // *word offset in the window*, alternates its request between the half
        // size and the full window so the event lands at each half boundary in
        // turn. Passing that alternation through unchanged asks this chip for
        // an event every 64 words instead of every 32 — the second refill then
        // arrives a whole half late and the transmitter walks into the guard
        // word planted by the first. Measured before this clamp: `guard_trips`
        // exactly equal to `frames` on every channel whose frame outgrew one
        // RAM window (8/16/100/256 LEDs), with `refill_lag_avg_words` a
        // comfortable 7.0 — the refills were not late, they were *asked for*
        // late.
        //
        // So the request is clamped to the channel's half size, which is the
        // only period that produces the boundary events the core expects. A
        // request smaller than a half (the core never makes one today) is
        // passed through, so the test hook that suppresses a threshold still
        // behaves as written.
        let half = (self.blocks.window_words(ch, BLOCK_WORDS) / 2) as u16;
        let period = if half == 0 { words } else { words.min(half) };
        // A note on what was tried here, so it is not tried again blind.
        // Writing `CH_TX_LIM` restarts the channel's entry counter, so the
        // core's once-per-refill call re-arms mid-frame — a plausible cause of
        // the multi-channel truncation `sweep_channels` measures (see the
        // README). Caching the value and skipping the redundant writes was
        // tried on silicon, both with and without a per-frame re-arm in
        // `start_tx`. **Neither fixed it**: channel 2 went clean and channel 1
        // got *worse* (10.0 -> 4.0 refills per frame, a guard skip on every
        // frame), so the rewrite shifts the failure without causing it. The
        // unconditional write is kept because it is the form the loopback
        // suite and the golden vectors were validated against.
        // SAFETY (register): `tx_lim` is a plain 9-bit field; the value is
        // masked to that width, so no reserved bits are disturbed. The PAC
        // marks the setter unsafe only because it cannot check the width.
        RMT::regs()
            .ch_tx_lim(ch as usize)
            .modify(|_, w| unsafe { w.tx_lim().bits(period & TX_LIM_MAX) });
    }

    #[inline]
    fn read_pos(&self, ch: u8) -> u16 {
        let window = self.blocks.window_words(ch, BLOCK_WORDS);
        if window == 0 {
            return 0;
        }
        let absolute = RMT::regs()
            .chstatus(ch as usize)
            .read()
            .mem_raddr_ex()
            .bits();
        // `mem_raddr_ex` counts from the start of the *whole* RMT RAM — same
        // absolute-offset semantics as the S3, different register name
        // (`CHnSTATUS` here, `CH_TX_STATUS` there); esp-hal's classic
        // `hw_offset()` subtracts the same term. The modulo keeps a reading
        // taken mid-wrap inside the window instead of panicking or aliasing.
        let base = self.blocks.window_start(ch, BLOCK_WORDS) as u16;
        absolute.wrapping_sub(base) % window as u16
    }

    fn start_tx(&self, ch: u8) {
        if ch as usize >= TX_CHANNELS {
            return;
        }
        let rmt = RMT::regs();
        let idx = ch as usize;

        // Drop causes left over from the previous frame so the first event of
        // this one is genuinely this one's.
        // SAFETY (register): `int_clr` is write-1-to-clear across its whole
        // width; the mask only names this channel's TX event bits.
        rmt.int_clr().write(|w| unsafe { w.bits(tx_event_mask(ch)) });

        // Reset the channel clock divider so the first pulse is a full tick
        // rather than the remainder of one already in progress. Unlike the
        // S3's shared `REF_CNT_RST` bitmask register, this is a per-channel
        // bit in CHnCONF1 (bit 16, PAC `ref_cnt_rst`).
        rmt.chconf1(idx).modify(|_, w| w.ref_cnt_rst().set_bit());
        rmt.chconf1(idx).modify(|_, w| w.ref_cnt_rst().clear_bit());

        // A window wider than one block extends into the *following* channels'
        // blocks; each of those blocks is owned via its own channel's
        // `mem_owner` bit, so hand every one of them to the transmitter —
        // esp-hal's classic `start_tx` does exactly this dance.
        for extra in 1..self.blocks.blocks(ch) {
            rmt.chconf1(idx + extra as usize)
                .modify(|_, w| w.mem_owner().clear_bit());
        }

        // Single-shot, wrapping around the window (wrap enable itself is the
        // global `APB_CONF` bit set once by `init_tx`): the driver keeps
        // refilling behind the read pointer and ends the frame with a STOP
        // word. `mem_rd_rst` rewinds the transmitter to word 0 of the window,
        // `apb_mem_rst` the APB side; both are write-to-trigger, there is no
        // `conf_update` on this chip, and `mem_owner` clear = transmitter owns
        // the RAM.
        rmt.chconf1(idx).modify(|_, w| {
            w.tx_conti_mode().clear_bit();
            w.mem_owner().clear_bit();
            w.mem_rd_rst().set_bit();
            w.apb_mem_rst().set_bit();
            w.tx_start().set_bit()
        });
    }

    fn stop_tx(&self, ch: u8) {
        if ch as usize >= TX_CHANNELS {
            return;
        }
        // The classic ESP32 has no immediate-stop bit
        // (`rmt.has_tx_immediate_stop` = false; the S3's `tx_stop` does not
        // exist in CHnCONF1). The only way to stop a transmitter is the way
        // esp-hal does it: fill the channel's whole RAM window with STOP
        // markers so the wrap-mode reader lands on one within a word. An
        // all-zero word is the STOP marker, so this doubles as leaving the
        // channel in the safest possible state — and it is harmless on an
        // already-idle channel, which the driver's cleanup path relies on.
        for word in 0..self.blocks.window_words(ch, BLOCK_WORDS) {
            if let Some(ptr) = ram_word(&self.blocks, ch, word) {
                // SAFETY: in-range, aligned RMT RAM pointer from `ram_word`;
                // volatile because the transmitter reads this memory
                // concurrently — that race is the mechanism, not a hazard: an
                // aligned u32 store is single-copy-atomic and every value the
                // reader can observe (old word or STOP) is a valid pulse word.
                unsafe { ptr.write_volatile(0) };
            }
        }
    }

    #[inline]
    fn take_interrupts(&self) -> InterruptFlags {
        let rmt = RMT::regs();
        // `int_st` is `int_raw & int_ena`, so causes this firmware never asked
        // for (RX events, other channels) cannot reach the driver.
        let status = rmt.int_st().read().bits();

        let mut end = 0u32;
        let mut error = 0u32;
        let mut threshold = 0u32;
        let mut pending = 0u32;
        // The interleaved bit layout has no contiguous per-event field to mask
        // out in one shift (the S3 does); eight compares are cheap next to the
        // register read itself.
        let mut ch = 0u8;
        while (ch as usize) < TX_CHANNELS {
            let one = 1u32 << ch;
            if status & int_tx_end_bit(ch) != 0 {
                end |= one;
                pending |= int_tx_end_bit(ch);
            }
            if status & int_err_bit(ch) != 0 {
                error |= one;
                pending |= int_err_bit(ch);
            }
            if status & int_thr_bit(ch) != 0 {
                threshold |= one;
                pending |= int_thr_bit(ch);
            }
            ch += 1;
        }

        if pending != 0 {
            // Acknowledge exactly what is being reported, in the same handler
            // pass — the driver's contract is that no cause is lost between
            // the read and the clear.
            // SAFETY (register): write-1-to-clear; `pending` is a subset of
            // the bits just read from `int_st`.
            rmt.int_clr().write(|w| unsafe { w.bits(pending) });
        }

        InterruptFlags {
            threshold,
            end,
            error,
        }
    }
}

/// One-time TX-side peripheral setup: enable wrap mode.
///
/// On the classic ESP32 wrap enable is the **global** `APB_CONF.mem_tx_wrap_en`
/// bit rather than a per-channel conf bit — every transmitting channel here
/// wants wrap, so it is set once at init instead of read-modified on every
/// `start_tx` (which would put an unnecessary shared-register RMW on the frame
/// start path). `apb_fifo_mask` (bit 0, 1 = direct RAM access rather than
/// FIFO) is already set by esp-hal's `Rmt::new`; the RAM probe toggles it and
/// restores it.
pub fn init_tx() {
    RMT::regs()
        .apb_conf()
        .modify(|_, w| w.mem_tx_wrap_en().set_bit());
}

/// Enable `tx_end`, `err` and `tx_thr_event` for `ch` in `INT_ENA`.
///
/// `chN_err` is the chip's combined TX/RX error bit; enabling it on a channel
/// this firmware transmits on cannot surface RX causes, because a channel is
/// TX-or-RX and this one is TX.
pub fn enable_tx_interrupts(ch: u8) {
    if ch as usize >= TX_CHANNELS {
        return;
    }
    RMT::regs().int_ena().modify(|_, w| {
        w.ch_tx_end(ch).set_bit();
        w.ch_err(ch).set_bit();
        w.ch_tx_thr_event(ch).set_bit()
    });
}

/// [`enable_tx_interrupts`] for every channel `blocks` makes available.
pub fn enable_tx_interrupts_for(blocks: &BlockPlan<TX_CHANNELS>) {
    for ch in 0..TX_CHANNELS as u8 {
        if blocks.is_available(ch) {
            enable_tx_interrupts(ch);
        }
    }
}

/// Mask every RMT interrupt cause this firmware ever enabled.
///
/// Call before releasing the peripheral. Dropping the last esp-hal `Channel`
/// takes the RMT's clock away, and an interrupt that arrives after that point
/// runs [`RmtHw::take_interrupts`] against a clock-gated peripheral — which on
/// the classic ESP32 stalls the bus access and wedges the CPU inside a
/// maximum-priority handler, with no output and no way out.
#[cfg_attr(
    not(any(feature = "test_loopback", feature = "diag")),
    allow(dead_code)
)]
pub fn disable_all_interrupts() {
    // SAFETY (register): `int_ena` is a plain enable mask; writing zero
    // disables every cause and disturbs nothing else.
    RMT::regs().int_ena().write(|w| unsafe { w.bits(0) });
}

/// Zero every word of `ch`'s RAM window.
///
/// An all-zero word is the STOP marker, so this leaves the channel in the
/// safest possible state: whatever happens, the transmitter stops at word 0.
// The RAM probe is only run by the demo build; the loopback harness proves the
// offset end-to-end instead (a wrong offset cannot decode a single frame).
#[cfg_attr(not(demo_build), allow(dead_code))]
pub fn clear_ram(blocks: &BlockPlan<TX_CHANNELS>, ch: u8) {
    for word in 0..blocks.window_words(ch, BLOCK_WORDS) {
        if let Some(ptr) = ram_word(blocks, ch, word) {
            // SAFETY: in-range, aligned RMT RAM pointer from `ram_word`;
            // volatile because the transmitter also reads this memory.
            unsafe { ptr.write_volatile(0) };
        }
    }
}

/// What [`probe_ram_address`] found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RamProbe {
    /// The word read straight back after a direct store — proves the address
    /// behaves like memory at all.
    pub direct_readback: u32,
    /// The word the *peripheral* deposited via its APB FIFO port, read through
    /// [`RAM_BASE`]. Equal to the sentinel only if `RAM_BASE` really is where
    /// the RMT keeps channel `ch`'s data.
    pub fifo_readback: u32,
}

impl RamProbe {
    /// Both halves of the probe agreed with the sentinels.
    #[cfg_attr(not(demo_build), allow(dead_code))]
    pub fn ok(&self, direct: u32, fifo: u32) -> bool {
        self.direct_readback == direct && self.fifo_readback == fifo
    }
}

/// Confirm on-chip that [`RAM_BASE`] is the RMT's channel RAM.
///
/// Two independent checks, same scheme as the S3's:
///
/// 1. store a sentinel through the computed pointer and read it back — the
///    address is writable memory rather than a read-only or absent window;
/// 2. clear `APB_CONF.apb_fifo_mask` (`0` = "access memory by FIFO"), write a
///    second sentinel to `CHnDATA`, and restore direct access. That write
///    goes through the peripheral's *own* address generator, so finding it at
///    `RAM_BASE + ch * 64` is the hardware agreeing with the constant. A wrong
///    offset fails this even though it would pass check 1 against any RAM.
///
/// (The classic ESP32's known FIFO erratum concerns *reads* through `CHnDATA`;
/// this probe only writes through it and reads back directly.)
///
/// Leaves the window zeroed and the peripheral back in direct-access mode.
/// Must be called while `ch` is idle.
#[cfg_attr(not(demo_build), allow(dead_code))]
pub fn probe_ram_address(
    blocks: &BlockPlan<TX_CHANNELS>,
    ch: u8,
    direct_sentinel: u32,
    fifo_sentinel: u32,
) -> RamProbe {
    let rmt = RMT::regs();
    let idx = ch as usize;

    let direct_readback = match ram_word(blocks, ch, 0) {
        Some(ptr) => {
            // SAFETY: in-range, aligned RMT RAM pointer; volatile so the store
            // and the load either side of it are actually performed.
            unsafe {
                ptr.write_volatile(direct_sentinel);
                ptr.read_volatile()
            }
        }
        None => 0,
    };

    // Rewind the APB write pointer to the start of the channel's window, then
    // hand the peripheral the FIFO sentinel.
    rmt.chconf1(idx).modify(|_, w| w.apb_mem_rst().set_bit());
    rmt.chconf1(idx).modify(|_, w| w.apb_mem_rst().clear_bit());
    rmt.apb_conf().modify(|_, w| w.apb_fifo_mask().clear_bit());
    // SAFETY (register): `CHnDATA` is a full-width data port; every bit
    // pattern is valid. The PAC marks it unsafe because it has no field
    // constraints to check.
    rmt.chdata(idx).write(|w| unsafe { w.bits(fifo_sentinel) });
    rmt.apb_conf().modify(|_, w| w.apb_fifo_mask().set_bit());

    let fifo_readback = match ram_word(blocks, ch, 0) {
        // SAFETY: as above.
        Some(ptr) => unsafe { ptr.read_volatile() },
        None => 0,
    };

    rmt.chconf1(idx).modify(|_, w| w.apb_mem_rst().set_bit());
    rmt.chconf1(idx).modify(|_, w| w.apb_mem_rst().clear_bit());
    clear_ram(blocks, ch);

    RamProbe {
        direct_readback,
        fifo_readback,
    }
}
