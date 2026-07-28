//! E4 — window overflow/underflow under JIT-emitted frames.
//!
//! The 64-entry physical AR file holds only a handful of call frames; deeper
//! chains raise WindowOverflow exceptions whose handlers (installed by
//! esp-hal's Xtensa runtime) spill frames to the per-frame base save areas,
//! and WindowUnderflow reloads them on return. Recursing to depth 100 through
//! hand-emitted ENTRY frames forces many spill/reload round-trips; the
//! arithmetic only survives if every one was correct.
//!
//! Blob A (`f(d) = d`):                Blob B (mixed, `f(d) = d + builtin(7)`):
//!   +0 lit: self                        +0 lit: self
//!   +4 code                             +4 lit: spike_builtin
//!                                       +8 code (base case CALLX8s the builtin)

use crate::e3::spike_builtin;
use crate::jitbuf::{isync, memw, JitBuf};

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

    .align   4
    .global  spike_recb_blob
spike_recb_blob:
    .word    spike_recb
    .word    spike_builtin
    .global  spike_recb
spike_recb:
    entry   a1, 32
    beqz    a2, .Lrecb_done
    l32r    a8, spike_recb_blob
    addi    a10, a2, -1
    callx8  a8
    addi    a2, a10, 1
    retw
.Lrecb_done:
    l32r    a8, spike_recb_blob + 4
    movi    a10, 7
    callx8  a8
    mov     a2, a10
    retw
"#
);

extern "C" {
    fn spike_rec(depth: u32) -> u32;
    fn spike_recb(depth: u32) -> u32;
}

/// Golden vector #3a: recursive stub, verbatim from objdump (single literal
/// slot; the on-disk layout matches the RAM layout, so encodings transfer).
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

/// Golden vector #3b: mixed recursive stub with builtin base case.
///
/// NOT a verbatim objdump copy: LLVM MC deduplicated the `.word spike_builtin`
/// literal against E3's pool (out of this blob!), so the on-disk reference is
/// not self-contained. Both L32Rs below are re-encoded for the two-slot RAM
/// layout — target = ((PC + 3) & !3) + (imm16 << 2):
///   l32r at +14: ((14+3)&!3)=16, slot +0  -> imm16 = -4 (0xfffc)
///   l32r at +29: ((29+3)&!3)=32, slot +4  -> imm16 = -7 (0xfff9)
pub const RECB_BLOB_BYTES: [u8; 44] = [
    0x00, 0x00, 0x00, 0x00, // +0  literal: self (patched at runtime)
    0x00, 0x00, 0x00, 0x00, // +4  literal: spike_builtin (patched at runtime)
    0x36, 0x41, 0x00, // +8  entry a1, 32
    0x16, 0xe2, 0x00, // +11 beqz a2, +14 (to +29, the base case)
    0x81, 0xfc, 0xff, // +14 l32r a8, <slot +0>  (re-encoded: imm16 = -4)
    0xa2, 0xc2, 0xff, // +17 addi a10, a2, -1
    0xe0, 0x08, 0x00, // +20 callx8 a8
    0x22, 0xca, 0x01, // +23 addi a2, a10, 1
    0x90, 0x00, 0x00, // +26 retw
    0x81, 0xf9, 0xff, // +29 l32r a8, <slot +4>  (re-encoded: imm16 = -7)
    0xa2, 0xa0, 0x07, // +32 movi a10, 7
    0xe0, 0x08, 0x00, // +35 callx8 a8
    0xa0, 0x2a, 0x20, // +38 mov a2, a10 (wide or)
    0x90, 0x00, 0x00, // +41 retw
];
pub const RECB_BLOB_ENTRY_OFFSET: usize = 8;

const DEPTH: u32 = 100;

fn current_sp() -> usize {
    let sp: usize;
    // SAFETY: reading a1 (the stack pointer) has no side effects.
    unsafe { core::arch::asm!("mov {0}, a1", out(reg) sp) };
    sp
}

pub fn run() {
    // E4A — toolchain-assembled references first.
    // SAFETY: complete windowed functions taking one u32.
    let a = unsafe { spike_rec(DEPTH) };
    let b = unsafe { spike_recb(DEPTH) };
    if a == DEPTH && b == DEPTH + 21 {
        esp_println::println!("E4A: PASS depth={DEPTH} result={a} mixed={b} sp={:#x}", current_sp());
    } else {
        esp_println::println!("E4A: FAIL result={a} mixed={b} expected={DEPTH}/{}", DEPTH + 21);
    }

    // E4 — RAM copies; self/builtin literals patched IN PLACE (the self
    // address only exists once the buffer address is known).
    let mut buf_a = JitBuf::new(&REC_BLOB_BYTES);
    let self_a = (buf_a.exec_addr() + REC_BLOB_ENTRY_OFFSET) as u32;
    buf_a.patch_u32(0, self_a);

    let builtin_addr = spike_builtin as extern "C" fn(u32) -> u32 as usize as u32;
    let mut buf_b = JitBuf::new(&RECB_BLOB_BYTES);
    let self_b = (buf_b.exec_addr() + RECB_BLOB_ENTRY_OFFSET) as u32;
    buf_b.patch_u32(0, self_b);
    buf_b.patch_u32(4, builtin_addr);

    memw();
    isync();

    // SAFETY: buffers hold complete windowed functions; entry past the pools.
    let fa: extern "C" fn(u32) -> u32 =
        unsafe { core::mem::transmute(buf_a.exec_addr() + REC_BLOB_ENTRY_OFFSET) };
    let fb: extern "C" fn(u32) -> u32 =
        unsafe { core::mem::transmute(buf_b.exec_addr() + RECB_BLOB_ENTRY_OFFSET) };
    let ra = fa(DEPTH);
    let rb = fb(DEPTH);
    if ra == DEPTH && rb == DEPTH + 21 {
        esp_println::println!("E4: PASS depth={DEPTH} result={ra} mixed={rb}");
    } else {
        esp_println::println!("E4: FAIL result={ra} mixed={rb} expected={DEPTH}/{}", DEPTH + 21);
    }
}
