//! ESP32-C6 RMT backend for [`lp_ws281x::RmtHw`].
//!
//! All the chip knowledge in this firmware lives here: the RMT RAM address, the
//! `CH_TX_CONF0` start/stop dance, the interrupt-register bit layout, and the
//! fact that `MEM_RADDR_EX` is an offset into the *whole* RMT RAM rather than
//! into the channel's own window. Everything else — what to write and when — is
//! [`lp_ws281x::Ws281xDriver`], which never sees a register name.
//!
//! # Provenance
//!
//! This is a cleanup-port of the author's own single-channel C6 driver in
//! lp2025 (`lp-fw/fw-esp32/src/output/rmt/`), which pokes the same registers
//! directly: `RMT::ptr() + 0x400` for the RAM, `ch_tx_conf0` for start/stop,
//! `ch_tx_lim.tx_lim` for the threshold, `ch_tx_status.mem_raddr_ex` for the
//! read pointer. What changed on the way into [`RmtHw`]:
//!
//! * the RAM pointer is per-channel and bounds-checked ([`ram_word`]) instead of
//!   a bare `base_ptr.add(i)` that assumed channel 0 owned all four blocks;
//! * `read_pos` subtracts the channel's window start — lp2025 could not have
//!   noticed the offset is absolute, because with `memsize(4)` on channel 0 the
//!   window start is 0 (see below);
//! * the interrupt read is one snapshot of `int_st` (`int_raw & int_ena`) with a
//!   matching clear, instead of reading `int_raw` and clearing a fixed mask —
//!   the old shape could drop a cause raised between the read and the write;
//! * `tx_lim`'s loop fields are left alone. lp2025 rewrote `tx_loop_cnt_en`,
//!   `loop_count_reset` and `tx_loop_num` on every threshold flip; the loop
//!   engine is off for a single-shot frame, so those writes were noise on the
//!   hottest path in the driver;
//! * the clock-divider reset (`ref_cnt_rst`) is new here — the S3 backend added
//!   it so the first pulse of a frame is a whole tick.
//!
//! Sources for every register fact below, in the order they settled it:
//! the `esp32c6` PAC 0.23.2 (`src/rmt/*.rs`, field offsets and widths),
//! `esp-metadata-generated` 0.4.0 (`rmt.ram_start`, `rmt.channel_ram_size`),
//! and esp-hal 1.1.1's own RMT driver for the `mem_raddr_ex` convention. All
//! MIT/Apache-2.0. No GPL source was consulted.
//!
//! # Where the RAM is
//!
//! The ESP32-C6 RMT RAM sits at **`RMT_BASE + 0x400` = `0x6000_6400`**, *not* at
//! the `+0x800` the ESP32-S3 uses. The C6 has four 48-word blocks against the
//! S3's eight, and the register block in front of them is correspondingly
//! smaller. `rmt.ram_start` for `esp32c6` in `esp-metadata-generated` v0.4.0 is
//! `1610638336` = `0x6000_6400`, and the PAC puts the peripheral itself at
//! `0x6000_6000`. [`probe_ram_address`] confirms it on silicon by making the
//! peripheral write a word through its own APB FIFO port.
//!
//! # Channel split — the biggest structural difference from the S3
//!
//! The C6 has **two TX channels (`CH0`, `CH1`) and two RX channels (`CH2`,
//! `CH3`)**, and the roles are fixed in hardware: `CH2`/`CH3` have no
//! `CH_TX_CONF0` at all. The S3 has 4 + 4. Both chips still lay the four/eight
//! 48-word blocks out contiguously from `ram_start` in channel-number order, so
//! TX channel `n`'s window starts at word `n * 48` on both.
//!
//! # Register names and layouts that differ from the S3
//!
//! * `INT_RAW`/`INT_ST`/`INT_ENA`/`INT_CLR` **interleave TX and RX**:
//!   `ch_tx_end` in bits 0..=1, `ch_rx_end` in 2..=3, `ch_tx_err` in 4..=5,
//!   `ch_rx_err` in 6..=7, `ch_tx_thr_event` in 8..=9, `ch_rx_thr_event` in
//!   10..=11, `ch_tx_loop` in 12..=13. The *shifts* happen to match the S3's
//!   (0/4/8/12) but the TX field is **two** bits wide, not four: the S3's
//!   `0b1111` mask would swallow the RX causes sitting next door — which
//!   esp-hal's blocking RX transaction polls out of `INT_RAW` — and the
//!   loopback harness would hang instead of failing loudly. Same accessor
//!   names, different geometry; see [`TX_CH_MASK`].
//! * `REF_CNT_RST` is **not** a uniform per-channel bitmask. The PAC names its
//!   bits `tx_ref_cnt_rst` (CH0, bit 0), `tx_ref_cnt_rst_ch1` (bit 1),
//!   `rx_ref_cnt_rst_ch2` (bit 2), `rx_ref_cnt_rst_ch3` (bit 3) — TX and RX
//!   interleaved again, so `1 << ch` is right here only because the two TX
//!   channels happen to be the two lowest bits. Written, not modified: the
//!   register is write-only.
//! * `CH_TX_STATUS.mem_raddr_ex` is **9 bits** (max 511) against the S3's 10 —
//!   enough for all 192 words of C6 RMT RAM, which is the tell that it is an
//!   absolute offset here too. `CH_TX_LIM.tx_lim` is 9 bits on both.

use esp_hal::peripherals::RMT;
use lp_ws281x::{BlockPlan, InterruptFlags, RmtHw};

/// TX channels the ESP32-C6 RMT exposes. `CH0`/`CH1` transmit; `CH2`/`CH3` are
/// receive-only (no `CH_TX_CONF0`) and are not addressed by this driver.
pub const TX_CHANNELS: usize = 2;

/// Words in one ESP32-C6 RMT memory block (`rmt.channel_ram_size` = 48 — the
/// same as the S3, unlike the classic ESP32's 64).
pub const BLOCK_WORDS: usize = 48;

/// Memory blocks given to each channel.
///
/// One block each is the interesting configuration and the shipped default: a
/// 48-word window halves into 24 words = exactly one LED, which is the tightest
/// refill deadline the hardware can pose (~30 µs at 800 kHz) *and* the only way
/// to get both outputs. Raising this to 2 leaves a single output — a channel's
/// window extends into the block of the channel above it, which then cannot
/// transmit at all (see [`lp_ws281x::BlockPlan`]).
///
/// lp2025 went the other way: `memsize(4)` on channel 0, which on this chip
/// reaches past `CH1` into the two **RX** blocks. That works only because that
/// firmware never receives.
pub const BLOCKS_PER_CHANNEL: u8 = 1;

/// The TX-side allocation of the RMT's two transmit memory blocks.
///
/// Validated at compile time: an overlapping or oversized plan is a build
/// error, not a runtime surprise. The *same* value is handed to the driver
/// (which refuses to configure an absorbed channel) and to [`C6Rmt`] (which
/// sizes and bounds its RAM window from it), so the two can never disagree.
pub const TX_BLOCKS: BlockPlan<TX_CHANNELS> = match BlockPlan::uniform(BLOCKS_PER_CHANNEL) {
    Ok(plan) => plan,
    Err(_) => panic!("BLOCKS_PER_CHANNEL does not divide the ESP32-C6's two TX memory blocks"),
};

/// RAM words a channel owns under [`TX_BLOCKS`], for the channels that have any.
///
/// Reported by the demo's `E1: MEASURE` line; the loopback harness reads the
/// same number out of the plan instead.
#[cfg_attr(any(feature = "test_loopback", feature = "test_stress"), allow(dead_code))]
pub const CHANNEL_WORDS: usize = BLOCK_WORDS * BLOCKS_PER_CHANNEL as usize;

/// Total TX-side RMT RAM, in words — the bound every pointer here respects.
///
/// Deliberately *not* the whole 192-word RAM: words 96.. belong to the two RX
/// channels, and the loopback harness has esp-hal receiving into them at the
/// same moment this driver is refilling.
const TX_RAM_WORDS: usize = BLOCK_WORDS * TX_CHANNELS;

/// Byte offset from the RMT peripheral base to the start of RMT RAM.
///
/// **`0x400` on the ESP32-C6.** See the module docs — the S3 value is `0x800`.
pub const RAM_OFFSET: usize = 0x400;

/// Base address of the ESP32-C6 RMT peripheral (`esp32c6` PAC 0.23.2:
/// `pub type RMT = crate::Periph<rmt::RegisterBlock, 0x6000_6000>`).
///
/// Cross-checked against `RMT::ptr()` at run time by [`probe_ram_address`], so a
/// PAC that moved the peripheral cannot silently pass.
pub const RMT_BASE: usize = 0x6000_6000;

/// Absolute address of the ESP32-C6 RMT RAM (`0x6000_6400`).
pub const RAM_BASE: usize = RMT_BASE + RAM_OFFSET;

/// Mask covering the two TX channels within one event field of `INT_*`.
///
/// **Two bits, not the S3's four** — bits 2..=3 of the `*_end` field are
/// `ch_rx_end`, not `ch2_tx_end`/`ch3_tx_end`. See the module docs.
const TX_CH_MASK: u32 = 0b11;

/// Bit offset of the `ch_tx_err` field within `INT_*`.
const ERR_SHIFT: u32 = 4;

/// Bit offset of the `ch_tx_thr_event` field within `INT_*`.
const THR_SHIFT: u32 = 8;

/// Bit offset of the `ch_tx_loop` field within `INT_*`.
const LOOP_SHIFT: u32 = 12;

/// Widest value `CH_TX_LIM.tx_lim` (bits 0..=8) can hold.
const TX_LIM_MAX: u16 = 0x1FF;

/// Pointer to word `word_idx` of channel `ch`'s RAM window under `blocks`, or
/// `None` if either index is out of range.
///
/// The bounds check costs one compare per word on the refill path and buys a
/// handler that can never scribble outside the TX half of the peripheral RAM,
/// whatever the caller does — which matters more here than on the S3, because
/// the words immediately above the bound are the RX capture buffers the
/// loopback harness is filling at the same time.
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
    // `TX_CHANNELS * BLOCK_WORDS` u32 words long; `index` was just bounded to
    // that range, so the result stays inside one allocated MMIO object and the
    // byte offset (< 512 B) cannot overflow an `isize`.
    Some(unsafe { (RAM_BASE as *mut u32).add(index) })
}

/// The seven register operations `lp-ws281x` needs, on the ESP32-C6.
///
/// Carries only the memory-block plan: everything else it addresses is
/// memory-mapped, so it is `const`-constructible and can live in a `static`
/// shared with the interrupt handler.
#[derive(Debug, Clone, Copy)]
pub struct C6Rmt {
    blocks: BlockPlan<TX_CHANNELS>,
}

impl C6Rmt {
    /// A backend handle for `blocks`. Touches no hardware.
    ///
    /// Pass the same plan to [`lp_ws281x::Ws281xDriver::with_blocks`].
    pub const fn new(blocks: BlockPlan<TX_CHANNELS>) -> Self {
        Self { blocks }
    }
}

impl Default for C6Rmt {
    fn default() -> Self {
        Self::new(TX_BLOCKS)
    }
}

impl RmtHw for C6Rmt {
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
        // Only `tx_lim` is touched. lp2025 also rewrote `tx_loop_cnt_en`,
        // `loop_count_reset` and `tx_loop_num` here; the loop engine is off for
        // a single-shot frame, so `modify` preserving them is both correct and
        // cheaper on the refill path.
        // SAFETY (register): `tx_lim` is a plain 9-bit field; the value is
        // masked to that width, so no reserved bits are disturbed. The PAC
        // marks the setter unsafe only because it cannot check the width.
        RMT::regs()
            .ch_tx_lim(ch as usize)
            .modify(|_, w| unsafe { w.tx_lim().bits(words & TX_LIM_MAX) });
    }

    #[inline]
    fn read_pos(&self, ch: u8) -> u16 {
        let window = self.blocks.window_words(ch, BLOCK_WORDS);
        if window == 0 {
            return 0;
        }
        let absolute = RMT::regs()
            .ch_tx_status(ch as usize)
            .read()
            .mem_raddr_ex()
            .bits();
        // `mem_raddr_ex` counts from the start of the *whole* RMT RAM, exactly
        // as on the S3 — esp-hal's `hw_offset()` subtracts
        // `channel * channel_ram_size` on this chip too. lp2025 read the field
        // raw, which was right only because its single channel was channel 0.
        // The modulo keeps a reading taken mid-wrap inside the window instead
        // of panicking or aliasing.
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
        // width; the mask only names this channel's four TX event bits and
        // never the RX bits interleaved between them.
        rmt.int_clr()
            .write(|w| unsafe { w.bits(tx_event_mask(ch)) });

        // Reset the channel clock divider so the first pulse is a full one
        // rather than the remainder of a tick already in progress.
        // SAFETY (register): `REF_CNT_RST` is write-only and self-clearing;
        // bits 0 and 1 are the two TX channels' dividers, so `1 << ch` is in
        // range for `ch < TX_CHANNELS` and the zeroes elsewhere in the word
        // leave the RX dividers alone.
        rmt.ref_cnt_rst().write(|w| unsafe { w.bits(1 << ch) });
        // SAFETY (register): releasing the reset again, same field.
        rmt.ref_cnt_rst().write(|w| unsafe { w.bits(0) });

        // Single-shot, wrapping around the window: the driver keeps refilling
        // behind the read pointer and ends the frame with a STOP word.
        rmt.ch_tx_conf0(idx).modify(|_, w| {
            w.tx_stop().clear_bit();
            w.tx_conti_mode().clear_bit();
            w.mem_tx_wrap_en().set_bit()
        });
        rmt.ch_tx_conf0(idx)
            .modify(|_, w| w.conf_update().set_bit());

        // `mem_rd_rst` rewinds the transmitter to word 0 of the window and
        // `apb_mem_rst` the APB side; both are write-to-trigger.
        rmt.ch_tx_conf0(idx).modify(|_, w| {
            w.mem_rd_rst().set_bit();
            w.apb_mem_rst().set_bit();
            w.tx_start().set_bit()
        });
        rmt.ch_tx_conf0(idx)
            .modify(|_, w| w.conf_update().set_bit());
    }

    fn stop_tx(&self, ch: u8) {
        if ch as usize >= TX_CHANNELS {
            return;
        }
        let rmt = RMT::regs();
        let idx = ch as usize;
        rmt.ch_tx_conf0(idx).modify(|_, w| w.tx_stop().set_bit());
        rmt.ch_tx_conf0(idx)
            .modify(|_, w| w.conf_update().set_bit());
    }

    #[inline]
    fn take_interrupts(&self) -> InterruptFlags {
        let rmt = RMT::regs();
        // `int_st` is `int_raw & int_ena`, so causes this firmware never asked
        // for cannot reach the driver. On this chip that guard is load-bearing
        // rather than tidy: the RX causes live *inside* the same fields, and
        // esp-hal's blocking receive polls them out of `int_raw` itself.
        let status = rmt.int_st().read().bits();

        // `ch_tx_end` is the lowest field, so it needs no shift.
        let end = status & TX_CH_MASK;
        let error = (status >> ERR_SHIFT) & TX_CH_MASK;
        let threshold = (status >> THR_SHIFT) & TX_CH_MASK;

        let pending = end | (error << ERR_SHIFT) | (threshold << THR_SHIFT);
        if pending != 0 {
            // Acknowledge exactly what is being reported, in the same handler
            // pass — the driver's contract is that no cause is lost between the
            // read and the clear, and (unlike lp2025's fixed mask) an RX bit
            // that came up in between is not collateral.
            // SAFETY (register): write-1-to-clear; `pending` is a subset of the
            // bits just read from `int_st`.
            rmt.int_clr().write(|w| unsafe { w.bits(pending) });
        }

        InterruptFlags {
            threshold,
            end,
            error,
        }
    }
}

/// The `INT_*` bits belonging to TX channel `ch`.
///
/// Four bits, spread one per event field — never a contiguous nibble, because
/// the RX channels' bits sit between them.
const fn tx_event_mask(ch: u8) -> u32 {
    let bit = 1u32 << ch;
    // ch_tx_end | ch_tx_err | ch_tx_thr_event | ch_tx_loop
    bit | (bit << ERR_SHIFT) | (bit << THR_SHIFT) | (bit << LOOP_SHIFT)
}

/// Enable `ch_tx_end`, `ch_tx_err` and `ch_tx_thr_event` for `ch`; leave
/// `ch_tx_loop` off, and never touch an RX enable.
pub fn enable_tx_interrupts(ch: u8) {
    if ch as usize >= TX_CHANNELS {
        return;
    }
    RMT::regs().int_ena().modify(|_, w| {
        w.ch_tx_end(ch).set_bit();
        w.ch_tx_err(ch).set_bit();
        w.ch_tx_thr_event(ch).set_bit();
        w.ch_tx_loop(ch).clear_bit()
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

/// Zero every word of `ch`'s RAM window.
///
/// An all-zero word is the STOP marker, so this leaves the channel in the
/// safest possible state: whatever happens, the transmitter stops at word 0.
// The RAM probe is only run by the demo build; the loopback harness proves the
// offset end-to-end instead (a wrong offset cannot decode a single frame).
#[cfg_attr(any(feature = "test_loopback", feature = "test_stress"), allow(dead_code))]
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
    /// `RMT::ptr()` as the PAC reports it — must equal [`RMT_BASE`], or the
    /// `+0x400` offset is being measured from the wrong place.
    pub peripheral_base: usize,
}

impl RamProbe {
    /// All three parts of the probe agreed with the expectations.
    #[cfg_attr(any(feature = "test_loopback", feature = "test_stress"), allow(dead_code))]
    pub fn ok(&self, direct: u32, fifo: u32) -> bool {
        self.direct_readback == direct
            && self.fifo_readback == fifo
            && self.peripheral_base == RMT_BASE
    }
}

/// Confirm on-chip that [`RAM_BASE`] is the RMT's channel RAM.
///
/// Three independent checks:
///
/// 1. the PAC's own peripheral base matches [`RMT_BASE`], so `+0x400` is being
///    added to the right number;
/// 2. store a sentinel through the computed pointer and read it back — the
///    address is writable memory rather than a read-only or absent window;
/// 3. clear `SYS_CONF.apb_fifo_mask` (`0` = "access memory by FIFO"), write a
///    second sentinel to `CH<n>DATA`, and restore direct access. That write goes
///    through the peripheral's *own* address generator, so finding it at
///    `RAM_BASE + ch * 48` is the hardware agreeing with the constant. A wrong
///    offset fails this even though it would pass check 2 against any RAM.
///
/// Leaves the window zeroed and the peripheral back in direct-access mode. Must
/// be called while `ch` is idle.
#[cfg_attr(any(feature = "test_loopback", feature = "test_stress"), allow(dead_code))]
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
    rmt.ch_tx_conf0(idx)
        .modify(|_, w| w.apb_mem_rst().set_bit());
    rmt.ch_tx_conf0(idx)
        .modify(|_, w| w.conf_update().set_bit());
    rmt.sys_conf().modify(|_, w| w.apb_fifo_mask().clear_bit());
    // SAFETY (register): `CH<n>DATA` is a full-width data port; every bit
    // pattern is valid. The PAC marks it unsafe because it has no field
    // constraints to check.
    rmt.chdata(idx).write(|w| unsafe { w.bits(fifo_sentinel) });
    rmt.sys_conf().modify(|_, w| w.apb_fifo_mask().set_bit());

    let fifo_readback = match ram_word(blocks, ch, 0) {
        // SAFETY: as above.
        Some(ptr) => unsafe { ptr.read_volatile() },
        None => 0,
    };

    rmt.ch_tx_conf0(idx)
        .modify(|_, w| w.apb_mem_rst().set_bit());
    rmt.ch_tx_conf0(idx)
        .modify(|_, w| w.conf_update().set_bit());
    clear_ram(blocks, ch);

    RamProbe {
        direct_readback,
        fifo_readback,
        peripheral_base: RMT::ptr() as usize,
    }
}
