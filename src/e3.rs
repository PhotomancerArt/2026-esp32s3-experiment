//! E3 — JIT code calling a Rust "builtin" via CALLX8 + L32R literal pool.
//!
//! The hot boundary of the real system: emitted shader code calling Rust
//! builtins across the windowed ABI. Forces the two things E2 dodged:
//! argument staging across the CALL8 rotation (caller a10+ -> callee a2+),
//! and 32-bit address materialization via an L32R literal pool (MOVI covers
//! only ±2048; L32R references are backward-only).
//!
//! Layout (identical in the flash reference and the RAM copy, so the encoded
//! L32R offset transfers verbatim):
//!
//! ```text
//! +0  literal slot: address of spike_builtin (patched at runtime in E3b)
//! +4  entry a1, 48
//!     l32r  a8, <slot@+0>
//!     movi  a10, 42        ; arg: our a10 = callee's a2 after its ENTRY
//!     callx8 a8            ; return address lands in our a8
//!     mov   a2, a10        ; callee's return (its a2) arrives in our a10
//!     retw
//! ```

use crate::jitbuf::{isync, memw, JitBuf};

/// The fake builtin — compiled windowed by the esp toolchain, like every real
/// lightplayer builtin will be.
#[no_mangle]
pub extern "C" fn spike_builtin(x: u32) -> u32 {
    x.wrapping_mul(3)
}

core::arch::global_asm!(
    r#"
    .section .text.spike_ref3, "ax"
    .align   4
    .global  spike_call_blob
spike_call_blob:
    .word    spike_builtin
    .global  spike_call_builtin
spike_call_builtin:
    entry   a1, 48
    l32r    a8, spike_call_blob
    movi    a10, 42
    callx8  a8
    mov     a2, a10
    retw
"#
);

extern "C" {
    fn spike_call_builtin() -> u32;
}

/// Golden vector #2: literal slot + code, from xtensa-esp32s3-elf-objdump
/// (recorded in FINDINGS.md). First 4 bytes are the literal slot (link-time
/// address of spike_builtin in the reference; patched at runtime in E3b).
pub const CALL_BLOB_BYTES: [u8; 22] = [
    0x00, 0x00, 0x00, 0x00, // +0  literal slot (patched at runtime)
    0x36, 0x61, 0x00, // +4  entry a1, 48    ; word 0x006136 (imm12 = 48>>3 = 6)
    0x81, 0xfe, 0xff, // +7  l32r a8, <-8>   ; word 0xfffe81, imm16 = -2 words
    0xa2, 0xa0, 0x2a, // +10 movi a10, 42    ; word 0x2aa0a2
    0xe0, 0x08, 0x00, // +13 callx8 a8       ; word 0x0008e0
    0xa0, 0x2a, 0x20, // +16 mov a2, a10     ; word 0x202aa0 (or a2, a10, a10)
    0x90, 0x00, 0x00, // +18 retw            ; word 0x000090
];

/// Offset of the executable code within the blob (past the literal slot).
pub const CALL_BLOB_ENTRY_OFFSET: usize = 4;

pub fn run() {
    // E3A — toolchain-assembled reference.
    // SAFETY: spike_call_builtin is a complete windowed function.
    let v = unsafe { spike_call_builtin() };
    if v == 126 {
        esp_println::println!("E3A: PASS result={v}");
    } else {
        esp_println::println!("E3A: FAIL result={v} expected=126");
    }

    // E3 — same blob in RAM, literal patched to the live builtin address.
    let mut bytes = CALL_BLOB_BYTES;
    let builtin_addr = spike_builtin as extern "C" fn(u32) -> u32 as usize as u32;
    bytes[0..4].copy_from_slice(&builtin_addr.to_le_bytes());
    let buf = JitBuf::new(&bytes);
    memw();
    isync();
    // SAFETY: buffer holds the literal slot + a complete windowed function;
    // entry point is the code start past the slot, at the I-bus alias.
    let f: extern "C" fn() -> u32 =
        unsafe { core::mem::transmute(buf.exec_addr() + CALL_BLOB_ENTRY_OFFSET) };
    let v = f();
    if v == 126 {
        esp_println::println!(
            "E3: PASS result={v} builtin_addr={builtin_addr:#x} lit_addr={:#x}",
            buf.write_addr()
        );
    } else {
        esp_println::println!("E3: FAIL result={v} expected=126");
    }
}
