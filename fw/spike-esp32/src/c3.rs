//! C3 — windowed ABI on LX6: CALLX8 into a Rust builtin via an L32R literal
//! pool, from dynamically written code (GV2 shape from FINDINGS.md).
//!
//! C3A first runs a toolchain-assembled reference (linked into flash) so an
//! ABI failure separates cleanly from a memory-model failure.

use crate::codemem::{isync, memw, CodeSpot, RegionKind, RTC_IRAM_BASE};

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

/// Golden vector #2 (GV2), verbatim from FINDINGS.md (LX7-assembled; C3 tests
/// that it runs unmodified on LX6). Literal slot at +0 (runtime-patched),
/// entry at +4: `entry a1,48; l32r a8,<-8>; movi a10,42; callx8 a8;
/// mov a2,a10; retw`.
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

/// I-bus base for the C3 blob in the chosen region.
fn c3_spot(kind: RegionKind) -> CodeSpot {
    let iram_base = match kind {
        RegionKind::Sram1(_) => 0x400B_0A00,
        RegionKind::Sram0 => 0x4009_C300,
        RegionKind::RtcFast => RTC_IRAM_BASE + 0x1A00,
    };
    CodeSpot::new(kind, iram_base)
}

pub fn run(kind: RegionKind) {
    // C3A — toolchain-assembled reference (flash-resident).
    // SAFETY: spike_call_builtin is a complete windowed function.
    let v = unsafe { spike_call_builtin() };
    if v == 126 {
        esp_println::println!("C3A: PASS result={v}");
    } else {
        esp_println::println!("C3A: FAIL result={v} expected=126");
    }

    // C3 — GV2 in the discovered region, literal patched to the live builtin.
    let spot = c3_spot(kind);
    let builtin_addr = spike_builtin as extern "C" fn(u32) -> u32 as usize as u32;
    esp_println::println!(
        "C3: probing GV2 region={} exec={:#x} builtin={builtin_addr:#x}",
        kind.name(),
        spot.exec_addr(CALL_BLOB_ENTRY_OFFSET)
    );
    spot.write_code(&CALL_BLOB_BYTES);
    spot.write_word(0, builtin_addr); // patch the literal slot
    memw();
    isync();
    // SAFETY: the spot holds GV2 (literal slot + a complete windowed
    // function); the entry point is the code start past the slot, at its
    // I-bus address.
    let f: extern "C" fn() -> u32 =
        unsafe { core::mem::transmute(spot.exec_addr(CALL_BLOB_ENTRY_OFFSET)) };
    let v = f();
    if v == 126 {
        esp_println::println!(
            "C3: PASS result={v} region={} builtin_addr={builtin_addr:#x}",
            kind.name()
        );
    } else {
        esp_println::println!("C3: FAIL result={v} expected=126");
    }
}
