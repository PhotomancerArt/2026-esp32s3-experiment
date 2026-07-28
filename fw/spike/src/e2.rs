//! E2 — execute hand-assembled windowed code from RAM.
//!
//! E2A: a `global_asm!` reference stub, assembled by the toolchain and linked
//! into flash .text — proves the windowed-ABI understanding with zero
//! memory-model questions in play.
//!
//! E2:  the same instruction bytes, as a `const` array copied into a heap
//! buffer at runtime and executed via the SRAM1 I-bus alias — proves the
//! memory model (write via D-bus, fetch via I-bus, barriers).

use crate::jitbuf::{isync, memw, JitBuf};

core::arch::global_asm!(
    r#"
    .section .text.spike_ref, "ax"
    .global  spike_stub42
    .align   4
    .literal_position
spike_stub42:
    entry   a1, 32
    movi    a2, 42
    retw
"#
);

extern "C" {
    fn spike_stub42() -> u32;
}

/// Golden vector #1: the exact bytes of `spike_stub42` above, from
/// xtensa-esp32s3-elf-objdump of the linked ELF (recorded in FINDINGS.md).
///
/// ```text
/// entry a1, 32    ; 36 41 00   (word 0x004136)
/// movi  a2, 42    ; 22 a0 2a   (word 0x2aa022 — assembler chose the wide form)
/// retw            ; 90 00 00   (word 0x000090 — wide form)
/// ```
pub const STUB42_BYTES: [u8; 9] = [0x36, 0x41, 0x00, 0x22, 0xa0, 0x2a, 0x90, 0x00, 0x00];

pub fn run() {
    // E2A — static reference stub.
    // SAFETY: spike_stub42 is a windowed (`entry`/`retw`) leaf function taking
    // no arguments; calling it via the windowed extern "C" ABI is well-formed.
    let v = unsafe { spike_stub42() };
    if v == 42 {
        esp_println::println!("E2A: PASS value={v}");
    } else {
        esp_println::println!("E2A: FAIL value={v} expected=42");
    }

    // E2 — same bytes from a heap buffer, executed via the I-bus alias,
    // with memw + isync barriers between write and call.
    let buf = JitBuf::new(&STUB42_BYTES);
    memw();
    isync();
    // SAFETY: buf holds a complete windowed function (entry/retw), 4-byte
    // aligned, and exec_addr is the I-bus alias of the just-written bytes.
    let f: extern "C" fn() -> u32 = unsafe { core::mem::transmute(buf.exec_addr()) };
    let v = f();
    if v == 42 {
        esp_println::println!(
            "E2: PASS value={v} write_addr={:#x} exec_addr={:#x} barriers=memw+isync",
            buf.write_addr(),
            buf.exec_addr()
        );
    } else {
        esp_println::println!("E2: FAIL value={v} expected=42");
    }

    // E2C — probe: fresh buffer, NO barriers between write and call.
    // SRAM1 is uncached so this is expected to work; recording the answer.
    let buf2 = JitBuf::new(&STUB42_BYTES);
    // SAFETY: same as above, minus barriers — that absence is the experiment.
    let f2: extern "C" fn() -> u32 = unsafe { core::mem::transmute(buf2.exec_addr()) };
    let v2 = f2();
    esp_println::println!(
        "E2C: {} value={v2} barriers=none",
        if v2 == 42 { "PASS" } else { "FAIL" }
    );

    // E2D — identity-execution probe (feature-gated: this is EXPECTED to
    // fault, since D-bus addresses are not instruction-fetchable; run once to
    // record what the fault looks like, then build without the feature).
    #[cfg(feature = "identity-probe")]
    {
        let buf3 = JitBuf::new(&STUB42_BYTES);
        esp_println::println!("E2D: probing identity exec at {:#x} (expect fault)", buf3.write_addr());
        // SAFETY: not safe — deliberately jumping to a non-fetchable address
        // to observe the exception. Feature-gated off in normal builds.
        let f3: extern "C" fn() -> u32 = unsafe { core::mem::transmute(buf3.write_addr()) };
        let v3 = f3();
        esp_println::println!("E2D: SURPRISE identity exec worked value={v3}");
    }
}
