//! C4 — window overflow/underflow on LX6 under JIT-emitted frames.
//!
//! Self-recursion to depth 100 through hand-emitted ENTRY frames (GV3a), plus
//! the mixed recursion + builtin base case (GV3b) — many WindowOverflow /
//! WindowUnderflow spill/reload round-trips through the classic runtime's
//! handlers (installed by xtensa-lx-rt, same crate as on S3). A small depth
//! sweep brackets the spill onset (the CALL8 model predicts first spill at
//! ~depth 6, when the 64-entry AR file is exhausted; the spill itself is
//! architecturally invisible to the program — correctness at every depth is
//! the observable).

use crate::c3::spike_builtin;
use crate::codemem::{isync, memw, CodeSpot, RegionKind, RTC_IRAM_BASE};

core::arch::global_asm!(
    r#"
    .section .text.spike_ref4, "ax"
    .align   4
    .global  spike_rec_blob
spike_rec_blob:
    .word    spike_rec
    .global  spike_rec
spike_rec:
    entry   a1, 32
    beqz    a2, .Lrec_done
    l32r    a8, spike_rec_blob
    addi    a10, a2, -1
    callx8  a8
    addi    a2, a10, 1
    retw
.Lrec_done:
    movi    a2, 0
    retw
"#
);

extern "C" {
    fn spike_rec(depth: u32) -> u32;
}

/// Golden vector #3a (GV3a), verbatim from FINDINGS.md: self-recursive
/// windowed stub, `f(d) = d`. Self literal at +0 (runtime-patched), entry +4.
pub const REC_BLOB_BYTES: [u8; 31] = [
    0x00, 0x00, 0x00, 0x00, // +0  literal: self (patched at runtime)
    0x36, 0x41, 0x00, // +4  entry a1, 32
    0x16, 0xe2, 0x00, // +7  beqz a2, +14 (to +25, the base case)
    0x81, 0xfd, 0xff, // +10 l32r a8, <slot +0>  (imm16 = -3 words)
    0xa2, 0xc2, 0xff, // +13 addi a10, a2, -1
    0xe0, 0x08, 0x00, // +16 callx8 a8
    0x22, 0xca, 0x01, // +19 addi a2, a10, 1
    0x90, 0x00, 0x00, // +22 retw
    0x22, 0xa0, 0x00, // +25 movi a2, 0
    0x90, 0x00, 0x00, // +28 retw
];
pub const REC_BLOB_ENTRY_OFFSET: usize = 4;

/// Golden vector #3b (GV3b), from FINDINGS.md: mixed recursion with a builtin
/// base case, `f(d) = d + builtin(7) = d + 21`. Two-slot pool (+0 self,
/// +4 builtin, both runtime-patched), entry +8. Constructed, not copied —
/// both L32Rs re-encoded on the S3 spike (imm16 = -4 and -7) because LLVM MC
/// deduped the reference's second literal out of the blob.
pub const RECB_BLOB_BYTES: [u8; 44] = [
    0x00, 0x00, 0x00, 0x00, // +0  literal: self (patched at runtime)
    0x00, 0x00, 0x00, 0x00, // +4  literal: spike_builtin (patched at runtime)
    0x36, 0x41, 0x00, // +8  entry a1, 32
    0x16, 0xe2, 0x00, // +11 beqz a2, +14 (to +29, the base case)
    0x81, 0xfc, 0xff, // +14 l32r a8, <slot +0>  (imm16 = -4)
    0xa2, 0xc2, 0xff, // +17 addi a10, a2, -1
    0xe0, 0x08, 0x00, // +20 callx8 a8
    0x22, 0xca, 0x01, // +23 addi a2, a10, 1
    0x90, 0x00, 0x00, // +26 retw
    0x81, 0xf9, 0xff, // +29 l32r a8, <slot +4>  (imm16 = -7)
    0xa2, 0xa0, 0x07, // +32 movi a10, 7
    0xe0, 0x08, 0x00, // +35 callx8 a8
    0xa0, 0x2a, 0x20, // +38 mov a2, a10 (wide or)
    0x90, 0x00, 0x00, // +41 retw
];
pub const RECB_BLOB_ENTRY_OFFSET: usize = 8;

const DEPTH: u32 = 100;

fn spots(kind: RegionKind) -> (CodeSpot, CodeSpot) {
    let (a, b) = match kind {
        RegionKind::Sram1(_) => (0x400B_0B00, 0x400B_0C00),
        RegionKind::Sram0 => (0x4009_C400, 0x4009_C500),
        RegionKind::RtcFast => (RTC_IRAM_BASE + 0x1B00, RTC_IRAM_BASE + 0x1C00),
    };
    (CodeSpot::new(kind, a), CodeSpot::new(kind, b))
}

fn current_sp() -> usize {
    let sp: usize;
    // SAFETY: reading a1 (the stack pointer) has no side effects.
    unsafe { core::arch::asm!("mov {0}, a1", out(reg) sp) };
    sp
}

pub fn run(kind: RegionKind) {
    // C4A — toolchain-assembled reference first.
    // SAFETY: spike_rec is a complete windowed function taking one u32.
    let a = unsafe { spike_rec(DEPTH) };
    if a == DEPTH {
        esp_println::println!("C4A: PASS depth={DEPTH} result={a} sp={:#x}", current_sp());
    } else {
        esp_println::println!("C4A: FAIL result={a} expected={DEPTH}");
    }

    // C4 — RAM copies; self/builtin literals patched in place (the self
    // address only exists once the spot is chosen).
    let (spot_a, spot_b) = spots(kind);
    esp_println::println!(
        "C4: probing GV3 region={} exec_a={:#x} exec_b={:#x}",
        kind.name(),
        spot_a.exec_addr(REC_BLOB_ENTRY_OFFSET),
        spot_b.exec_addr(RECB_BLOB_ENTRY_OFFSET)
    );
    spot_a.write_code(&REC_BLOB_BYTES);
    spot_a.write_word(0, spot_a.exec_addr(REC_BLOB_ENTRY_OFFSET) as u32);

    let builtin_addr = spike_builtin as extern "C" fn(u32) -> u32 as usize as u32;
    spot_b.write_code(&RECB_BLOB_BYTES);
    spot_b.write_word(0, spot_b.exec_addr(RECB_BLOB_ENTRY_OFFSET) as u32);
    spot_b.write_word(1, builtin_addr);

    memw();
    isync();

    // SAFETY: both spots hold complete windowed functions; entry past the
    // literal pools, at their I-bus addresses.
    let fa: extern "C" fn(u32) -> u32 =
        unsafe { core::mem::transmute(spot_a.exec_addr(REC_BLOB_ENTRY_OFFSET)) };
    let fb: extern "C" fn(u32) -> u32 =
        unsafe { core::mem::transmute(spot_b.exec_addr(RECB_BLOB_ENTRY_OFFSET)) };

    // Depth sweep across the predicted spill onset (~6): any wrong result
    // pinpoints the depth where spill/reload first misbehaves.
    let mut sweep_ok = true;
    for d in [1u32, 2, 4, 6, 8, 10, 12, 16, 32] {
        let r = fa(d);
        if r != d {
            esp_println::println!("C4: FAIL sweep depth={d} result={r}");
            sweep_ok = false;
        }
    }
    if sweep_ok {
        esp_println::println!(
            "C4: MEASURE sweep=1..32 ok=true (spill onset ~6 not directly observable; all depths correct)"
        );
    }

    let ra = fa(DEPTH);
    let rb = fb(DEPTH);
    if ra == DEPTH && rb == DEPTH + 21 {
        esp_println::println!("C4: PASS depth={DEPTH} result={ra} mixed={rb} region={}", kind.name());
    } else {
        esp_println::println!("C4: FAIL result={ra} mixed={rb} expected={DEPTH}/{}", DEPTH + 21);
    }
}
