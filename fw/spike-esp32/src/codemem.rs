//! Classic-ESP32 (LX6) code-memory model: regions, address rules, code writer.
//!
//! Unlike the ESP32-S3 (where all internal SRAM is dual-mapped at a uniform
//! `+0x6F_0000` alias and "the heap is executable"), the classic chip's SRAM
//! blocks differ per-bus (esp-hal `ld/esp32/memory.x` + ESP32 TRM):
//!
//! - **SRAM0** `0x4008_0400..0x400A_0000` (iram_seg): instruction bus; whether
//!   the data bus can store to it at all (and at what granularity) is C2c's
//!   question.
//! - **SRAM1** DRAM `0x3FFE_0000..0x4000_0000` ↔ IRAM `0x400A_0000..0x400C_0000`:
//!   dual-mapped but documented *mirrored*; the exact rule is C2b's question
//!   (H1 linear vs H2 word-reversed).
//! - **RTC fast** DRAM `0x3FF8_0000` ↔ IRAM `0x400C_0000`, 8KB, RWX, expected
//!   clean 1:1 (C2a).
//!
//! Everything here is keyed on the **I-bus layout**: a [`CodeSpot`] is "word i
//! of the code lives at I-bus address `iram_base + 4*i`", and each region kind
//! knows how to compute the *write* address for that word. That formulation
//! absorbs a word-reversed SRAM1 mapping transparently (the writer just walks
//! DRAM downward).

/// RTC fast memory, D-bus view.
pub const RTC_DRAM_BASE: usize = 0x3FF8_0000;
/// RTC fast memory, I-bus view (PRO_CPU only on classic ESP32).
pub const RTC_IRAM_BASE: usize = 0x400C_0000;
/// RTC fast memory length (8KB).
pub const RTC_LEN: usize = 0x2000;

/// SRAM1 D-bus window.
pub const SRAM1_DRAM_BASE: usize = 0x3FFE_0000;
/// SRAM1 I-bus window start.
pub const SRAM1_IRAM_BASE: usize = 0x400A_0000;
/// SRAM1 I-bus window end (exclusive).
pub const SRAM1_IRAM_END: usize = 0x400C_0000;

/// SRAM0 (iram_seg) bounds — I-bus only.
pub const SRAM0_BASE: usize = 0x4008_0400;
/// SRAM0 end (exclusive).
pub const SRAM0_END: usize = 0x400A_0000;

/// How SRAM1's two views correspond (C2b's discovery).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sram1Rule {
    /// H1: `iram = 0x400A_0000 + (dram - 0x3FFE_0000)`.
    Linear,
    /// H2: `iram = 0x400B_FFFC - (dram - 0x3FFE_0000)`, word granularity —
    /// the two windows run in opposite directions.
    WordMirrored,
}

/// I-bus address of the word written at D-bus address `dram` under `rule`.
pub fn sram1_iram_for_dram(rule: Sram1Rule, dram: usize) -> usize {
    match rule {
        Sram1Rule::Linear => SRAM1_IRAM_BASE + (dram - SRAM1_DRAM_BASE),
        Sram1Rule::WordMirrored => (SRAM1_IRAM_END - 4) - (dram - SRAM1_DRAM_BASE),
    }
}

/// A code-capable memory region kind.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RegionKind {
    /// RTC fast RAM: write via D-bus at `-0xC40000` from the I-bus address.
    RtcFast,
    /// SRAM0: write directly through the I-bus address (if C2c says we can).
    Sram0,
    /// SRAM1: write via the D-bus window under the discovered rule.
    Sram1(Sram1Rule),
}

impl RegionKind {
    pub fn name(self) -> &'static str {
        match self {
            RegionKind::RtcFast => "rtc_fast",
            RegionKind::Sram0 => "sram0",
            RegionKind::Sram1(Sram1Rule::Linear) => "sram1_linear",
            RegionKind::Sram1(Sram1Rule::WordMirrored) => "sram1_word_mirrored",
        }
    }
}

/// A placement for dynamically written code: word `i` is fetchable at I-bus
/// address `iram_base + 4*i`.
#[derive(Clone, Copy)]
pub struct CodeSpot {
    pub kind: RegionKind,
    /// I-bus address of code word 0. Must be 4-byte aligned.
    pub iram_base: usize,
}

impl CodeSpot {
    pub fn new(kind: RegionKind, iram_base: usize) -> CodeSpot {
        assert!(iram_base % 4 == 0, "code spot must be word-aligned");
        CodeSpot { kind, iram_base }
    }

    /// The address to *write* word `i` through.
    pub fn write_addr(&self, i: usize) -> usize {
        let iram = self.iram_base + 4 * i;
        match self.kind {
            RegionKind::RtcFast => RTC_DRAM_BASE + (iram - RTC_IRAM_BASE),
            RegionKind::Sram0 => iram,
            RegionKind::Sram1(rule) => match rule {
                Sram1Rule::Linear => SRAM1_DRAM_BASE + (iram - SRAM1_IRAM_BASE),
                // Inverse of H2: dram = 0x3FFE_0000 + (0x400B_FFFC - iram).
                Sram1Rule::WordMirrored => SRAM1_DRAM_BASE + ((SRAM1_IRAM_END - 4) - iram),
            },
        }
    }

    /// Volatile word write of code word `i`.
    pub fn write_word(&self, i: usize, w: u32) {
        let addr = self.write_addr(i) as *mut u32;
        // SAFETY: addr is a word-aligned address inside a free scratch area of
        // the region (callers pick bases clear of linker-placed sections).
        unsafe { addr.write_volatile(w) };
    }

    /// Volatile word read-back of code word `i` (through the write view).
    pub fn read_word(&self, i: usize) -> u32 {
        let addr = self.write_addr(i) as *const u32;
        // SAFETY: same address validity argument as `write_word`.
        unsafe { addr.read_volatile() }
    }

    /// Copy `bytes` into the spot as word-aligned 32-bit volatile writes,
    /// zero-padded to a word multiple (padding sits after the final `retw`,
    /// never executed). Returns the number of words written.
    pub fn write_code(&self, bytes: &[u8]) -> usize {
        let words = bytes.len().div_ceil(4);
        for i in 0..words {
            let mut w = [0u8; 4];
            let start = i * 4;
            let end = (start + 4).min(bytes.len());
            w[..end - start].copy_from_slice(&bytes[start..end]);
            self.write_word(i, u32::from_le_bytes(w));
        }
        words
    }

    /// I-bus entry address for code whose entry point is `offset` bytes in.
    pub fn exec_addr(&self, offset: usize) -> usize {
        self.iram_base + offset
    }
}

/// Instruction-fetch pipeline barrier.
pub fn isync() {
    // SAFETY: isync has no operands and no memory safety impact.
    unsafe { core::arch::asm!("isync") };
}

/// Memory-ordering barrier.
pub fn memw() {
    // SAFETY: memw has no operands and no memory safety impact.
    unsafe { core::arch::asm!("memw") };
}
