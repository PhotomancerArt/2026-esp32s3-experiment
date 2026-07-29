//! Classic-ESP32 (LX6) payload code memory: a fixed SRAM1 region written
//! through the word-**mirrored** D-bus view.
//!
//! Unlike the S3 (uniform `+0x6F_0000` alias, "the heap is executable"), the
//! classic chip's heap (SRAM2/dram_seg) has no I-bus view at all — executing a
//! D-bus address faults with EXCCAUSE=2 (FINDINGS C2g). Dynamically written
//! code must go to one of the measured regions (FINDINGS C2); this runner uses
//! **SRAM1**, whose dual mapping is word-mirrored (C2b, all 5 sentinels):
//!
//! ```text
//! iram = 0x400B_FFFC − (dram − 0x3FFE_0000)     (word granularity)
//! ```
//!
//! i.e. the two windows run in opposite directions: writing I-bus-contiguous
//! code means walking the D-bus **downward** word by word. Everything here is
//! keyed on the I-bus layout — word `i` of the payload is fetchable at
//! `CODE_IBUS_BASE + 4*i` — and the write address is computed per word, which
//! absorbs the mirroring in one line of address math (the `CodeSpot` shape
//! from `fw/spike-esp32`, hardware-proven in C2/C3/C4). Bytes within each
//! little-endian 32-bit word are verbatim — no byte swap.
//!
//! ## The region (MUST match `lp-xt-emu`'s `BoardProfile::esp32()`)
//!
//! D-bus `0x3FFE_8000` + `0x1_7000` (92 KiB) ⇒ I-bus image
//! `0x400A_1000..0x400B_8000`, code word 0 at I-bus `0x400A_1000` (= the
//! D-bus *last* word `0x3FFF_EFFC`; the write walk ends at the D-bus base).
//! The emulator's classic profile models exactly this region; a mismatch
//! would silently break dual-run (P5/P6).
//!
//! The D-bus range sits inside esp-hal's `dram2_seg` (`0x3FFE_7E30 +
//! 98768`), where the only linkable section is `.dram2_uninit` — which this
//! firmware does not use — and clear of the ROM data/stack reservations lower
//! in SRAM1. `.data`/`.bss`/stack/heap all live in `dram_seg` (SRAM2, below
//! `0x3FFE_0000`), so nothing else touches the region.

use xt_runner_core::{CodeMem, LoadError};

/// SRAM1 D-bus window base (the mirrored rule's D-bus origin).
pub const SRAM1_DRAM_BASE: usize = 0x3FFE_0000;
/// I-bus address of the word at `SRAM1_DRAM_BASE` (the mirrored rule's top).
pub const SRAM1_IRAM_TOP: usize = 0x400B_FFFC;

/// D-bus base of the payload code region.
pub const CODE_DBUS_BASE: usize = 0x3FFE_8000;
/// Code region length in bytes (92 KiB).
pub const CODE_REGION_LEN: usize = 0x0001_7000;
/// I-bus address of payload byte 0 — the *lowest* I-bus address of the
/// region's executable image, which under the mirrored rule is the image of
/// the D-bus *last* word.
pub const CODE_IBUS_BASE: usize =
    SRAM1_IRAM_TOP - ((CODE_DBUS_BASE + CODE_REGION_LEN - 4) - SRAM1_DRAM_BASE);

// Pin the derived base so a region change that breaks emulator parity is loud:
// lp-xt-emu's BoardProfile::esp32() computes code_ibus_base() = 0x400A_1000
// from the same numbers.
const _: () = assert!(CODE_IBUS_BASE == 0x400A_1000);
const _: () = assert!(CODE_DBUS_BASE % 4 == 0 && CODE_REGION_LEN % 4 == 0);

/// The fixed SRAM1 code region. Zero-sized: the region is static, `load`
/// writes it in place. Fixed (not heap-backed) because the classic heap is
/// not executable — which is exactly why `capacity()`/`TooLarge` are
/// load-bearing here.
pub struct Sram1CodeMem;

impl Sram1CodeMem {
    pub fn new() -> Self {
        Sram1CodeMem
    }

    /// The D-bus address to write code word `i` through: the inverse of the
    /// mirrored rule at `iram = CODE_IBUS_BASE + 4*i`. Walks downward from
    /// `0x3FFF_EFFC` (i = 0) to `CODE_DBUS_BASE` (the last word).
    fn write_addr(i: usize) -> usize {
        let iram = CODE_IBUS_BASE + 4 * i;
        SRAM1_DRAM_BASE + (SRAM1_IRAM_TOP - iram)
    }
}

impl CodeMem for Sram1CodeMem {
    fn load(&mut self, code: &[u8]) -> Result<usize, LoadError> {
        if code.len() > CODE_REGION_LEN {
            return Err(LoadError::TooLarge {
                len: code.len(),
                capacity: CODE_REGION_LEN,
            });
        }
        // Word-aligned volatile writes, little-endian words verbatim,
        // zero-padded to a word multiple (padding sits after the final retw,
        // never executed). Word writes are the safe default on this chip
        // (mandatory on SRAM0's word-only bus; harmless on SRAM1).
        let words = code.len().div_ceil(4);
        for i in 0..words {
            let mut w = [0u8; 4];
            let start = i * 4;
            let end = (start + 4).min(code.len());
            w[..end - start].copy_from_slice(&code[start..end]);
            let addr = Self::write_addr(i) as *mut u32;
            // SAFETY: addr is a word-aligned D-bus address inside the fixed
            // code region (i < words <= CODE_REGION_LEN/4, so the walk stays
            // within CODE_DBUS_BASE..CODE_DBUS_BASE+CODE_REGION_LEN), which
            // the linker never places sections in (see module docs).
            unsafe { addr.write_volatile(u32::from_le_bytes(w)) };
        }
        Ok(CODE_IBUS_BASE)
    }

    fn sync(&mut self) {
        // Belt-and-braces: C2n measured that fresh SRAM1 code executes with no
        // barriers (internal SRAM is uncached), but the cost is nil.
        // SAFETY: memw/isync have no operands and no memory-safety impact.
        unsafe {
            core::arch::asm!("memw");
            core::arch::asm!("isync");
        }
    }

    fn capacity(&self) -> usize {
        CODE_REGION_LEN
    }
}
