//! Conformance corpus + dual-run harness.
//!
//! Every case runs on `lp-xt-emu`. When `XT_DEVICE_PORT` is set, each
//! position-independent case *also* runs on the ESP32-S3 via `xt-runner-client`
//! and the two outcomes are asserted equal (result value, and crash
//! classification on faults). Without a device the emulator still checks each
//! case against its known answer.
//!
//! Hardware tests share ONE board — run single-threaded:
//!   XT_DEVICE_PORT=/dev/cu.usbmodem1101 cargo test -p lp-xt-emu -- --test-threads=1 --nocapture
//!
//! All payload bytes are objdump-derived (see the scratch `.s` sources and
//! FINDINGS.md), never hand-recalled.

use lp_xt_emu::emu::{CODE_DBUS_BASE, RunOutcome as EmuOutcome};
use lp_xt_emu::memory::Memory;
use lp_xt_emu::{Emulator, TrapKind};

use xt_runner_client::{RunOutcome as HwOutcome, Runner};
use xt_runner_proto::CrashKind;

// ---------------------------------------------------------------------------
// Corpus (all bytes assembler-derived; entry at offset 0 unless noted).
// ---------------------------------------------------------------------------

/// GV1 `spike_stub42`: entry a1,32; movi a2,42; retw. f(_) = 42.
const STUB42: &[u8] = &[0x36, 0x41, 0x00, 0x22, 0xa0, 0x2a, 0x90, 0x00, 0x00];
/// identity: entry a1,32; retw. f(a) = a (arg arrives in a2).
const IDENTITY: &[u8] = &[0x36, 0x41, 0x00, 0x90, 0x00, 0x00];
/// arith: f(a) = 3*(a+5) - 2 = 3a + 13.
const ARITH: &[u8] = &[
    0x36, 0x41, 0x00, 0x22, 0xc2, 0x05, 0x20, 0x32, 0x90, 0x22, 0xc3, 0xfe, 0x90, 0x00, 0x00,
];
/// loadstore: store arg and 100 to the stack, reload, add. f(a) = a + 100.
const LOADSTORE: &[u8] = &[
    0x36, 0x41, 0x00, 0x22, 0x61, 0x00, 0x32, 0xa0, 0x64, 0x32, 0x61, 0x01, 0x42, 0x21, 0x00, 0x52,
    0x21, 0x01, 0x50, 0x24, 0x80, 0x90, 0x00, 0x00,
];
/// branchdir: bgei a2,10 forward. f(a) = if a >= 10 { 2 } else { 1 }.
const BRANCHDIR: &[u8] = &[
    0x36, 0x41, 0x00, 0xe6, 0x92, 0x05, 0x22, 0xa0, 0x01, 0x90, 0x00, 0x00, 0x22, 0xa0, 0x02, 0x90,
    0x00, 0x00,
];
/// sumloop: backward `j` loop summing 1..=a. f(a) = a*(a+1)/2.
const SUMLOOP: &[u8] = &[
    0x36, 0x41, 0x00, 0x32, 0xa0, 0x00, 0x20, 0x42, 0x20, 0x16, 0x84, 0x00, 0x40, 0x33, 0x80, 0x42,
    0xc4, 0xff, 0xc6, 0xfc, 0xff, 0x30, 0x23, 0x20, 0x90, 0x00, 0x00,
];
/// rec8: PC-relative call8 self-recursion (position-independent). f(d) = d.
const REC8: &[u8] = &[
    0x36, 0x41, 0x00, 0x16, 0xb2, 0x00, 0xa2, 0xc2, 0xff, 0x65, 0xff, 0xff, 0x22, 0xca, 0x01, 0x90,
    0x00, 0x00, 0x22, 0xa0, 0x00, 0x90, 0x00, 0x00,
];
/// rec12: PC-relative call12 self-recursion (arg in a14). f(d) = d.
const REC12: &[u8] = &[
    0x36, 0x61, 0x00, 0x16, 0xb2, 0x00, 0xe2, 0xc2, 0xff, 0x75, 0xff, 0xff, 0x22, 0xce, 0x01, 0x90,
    0x00, 0x00, 0x22, 0xa0, 0x00, 0x90, 0x00, 0x00,
];
/// entry a1,32; ill  — raises IllegalInstruction.
const CRASH_ILL: &[u8] = &[0x36, 0x41, 0x00, 0x00, 0x00, 0x00];
/// entry a1,32; j .  — infinite self-loop (watchdog / step-budget timeout).
const HANG: &[u8] = &[0x36, 0x41, 0x00, 0x06, 0xff, 0xff];

// --- emu-only golden vectors (callx8 + l32r; position-dependent) ---

/// mul3 windowed builtin: entry a1,16; addx2 a2,a2,a2; retw. f(x) = 3x.
const MUL3: &[u8] = &[0x36, 0x21, 0x00, 0x20, 0x22, 0x90, 0x90, 0x00, 0x00];
/// GV2 `spike_call_blob`: literal slot + callx8 builtin(42). entry at +4.
const CALL_BLOB: &[u8] = &[
    0x00, 0x00, 0x00, 0x00, 0x36, 0x61, 0x00, 0x81, 0xfe, 0xff, 0xa2, 0xa0, 0x2a, 0xe0, 0x08, 0x00,
    0xa0, 0x2a, 0x20, 0x90, 0x00, 0x00,
];
/// GV3a `spike_rec`: callx8 + l32r self-recursion. Self literal at +0, entry +4.
const REC_BLOB: &[u8] = &[
    0x00, 0x00, 0x00, 0x00, 0x36, 0x41, 0x00, 0x16, 0xe2, 0x00, 0x81, 0xfd, 0xff, 0xa2, 0xc2, 0xff,
    0xe0, 0x08, 0x00, 0x22, 0xca, 0x01, 0x90, 0x00, 0x00, 0x22, 0xa0, 0x00, 0x90, 0x00, 0x00,
];
/// GV3b `spike_recb`: recursion + builtin base case. Self +0, builtin +4, entry +8.
const RECB_BLOB: &[u8] = &[
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x41, 0x00, 0x16, 0xe2, 0x00, 0x81, 0xfc,
    0xff, 0xa2, 0xc2, 0xff, 0xe0, 0x08, 0x00, 0x22, 0xca, 0x01, 0x90, 0x00, 0x00, 0x81, 0xf9, 0xff,
    0xa2, 0xa0, 0x07, 0xe0, 0x08, 0x00, 0xa0, 0x2a, 0x20, 0x90, 0x00, 0x00,
];

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Expected known answer for a case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Expect {
    Ok(u32),
    Crash(TrapKind),
}

fn emu_run(code: &[u8], entry_offset: u32, arg: u32) -> EmuOutcome {
    let mut emu = Emulator::new();
    emu.run(code, entry_offset, arg)
}

/// Assert the emulator matches `expect`, and — when a device is present — that
/// the hardware agrees with the emulator (value and crash class).
fn dual_run(
    device: &mut Option<Runner>,
    seq: u32,
    name: &str,
    code: &[u8],
    entry_offset: u32,
    arg: u32,
    expect: Expect,
) {
    let emu = emu_run(code, entry_offset, arg);

    // 1) Emulator vs known answer.
    match (expect, emu) {
        (Expect::Ok(v), EmuOutcome::Ok(g)) => {
            assert_eq!(g, v, "[{name}] emu result mismatch (arg={arg})");
        }
        (Expect::Crash(k), EmuOutcome::Trap(t)) => {
            assert_eq!(t.kind, k, "[{name}] emu trap kind mismatch: {t:?}");
        }
        (exp, got) => panic!("[{name}] emu outcome {got:?} != expected {exp:?} (arg={arg})"),
    }

    // 2) Emulator vs hardware (position-independent cases only).
    let Some(dev) = device.as_mut() else {
        return;
    };
    let hw = dev
        .load_exec(seq, code.to_vec(), entry_offset, arg)
        .unwrap_or_else(|e| panic!("[{name}] device load_exec failed: {e}"));
    match (emu, hw) {
        (EmuOutcome::Ok(e), HwOutcome::Ok(h)) => {
            assert_eq!(e, h, "[{name}] EMU vs HW result diff (arg={arg})");
        }
        (EmuOutcome::Trap(t), HwOutcome::Crash(r)) => {
            let hw_kind = match r.kind {
                CrashKind::Timeout => TrapKind::Timeout,
                _ => TrapKind::Exception,
            };
            assert_eq!(
                t.kind, hw_kind,
                "[{name}] EMU vs HW crash-class diff: emu={:?} hw={:?}",
                t, r
            );
            eprintln!("[{name}] crash agree: emu={t:?} hw={r:?}");
        }
        (e, h) => panic!("[{name}] EMU vs HW outcome diff: emu={e:?} hw={h:?} (arg={arg})"),
    }
}

fn device() -> Option<Runner> {
    match Runner::from_env() {
        None => {
            eprintln!("XT_DEVICE_PORT unset — emulator-only (no hardware conformance)");
            None
        }
        Some(Ok(r)) => Some(r),
        Some(Err(e)) => panic!("failed to open device: {e}"),
    }
}

/// Patch a little-endian u32 into `code` at `off`.
fn patch_u32(code: &mut [u8], off: usize, val: u32) {
    code[off..off + 4].copy_from_slice(&val.to_le_bytes());
}

/// I-bus entry address the emulator will place `code` at.
fn emu_entry(entry_offset: u32) -> u32 {
    Memory::ibus_alias(CODE_DBUS_BASE).wrapping_add(entry_offset)
}

// ---------------------------------------------------------------------------
// The one #[test] entry point: runs the whole corpus sequentially so a single
// board is never driven concurrently.
// ---------------------------------------------------------------------------

#[test]
fn corpus_dual_run() {
    let mut dev = device();
    let mut seq = 1u32;
    let mut next = || {
        seq += 1;
        seq
    };

    // --- basic value-returning cases (dual-run capable) ---
    dual_run(&mut dev, next(), "stub42", STUB42, 0, 0, Expect::Ok(42));
    dual_run(&mut dev, next(), "stub42-argignored", STUB42, 0, 999, Expect::Ok(42));
    dual_run(&mut dev, next(), "identity", IDENTITY, 0, 1234, Expect::Ok(1234));
    dual_run(&mut dev, next(), "identity-0", IDENTITY, 0, 0, Expect::Ok(0));

    // arithmetic: f(a) = 3a + 13
    for a in [0u32, 1, 7, 1000] {
        dual_run(&mut dev, next(), "arith", ARITH, 0, a, Expect::Ok(3 * a + 13));
    }

    // load/store round-trip: f(a) = a + 100
    for a in [0u32, 5, 42, 100000] {
        dual_run(&mut dev, next(), "loadstore", LOADSTORE, 0, a, Expect::Ok(a + 100));
    }

    // branches both directions: f(a) = if a>=10 {2} else {1}
    dual_run(&mut dev, next(), "branch-nottaken", BRANCHDIR, 0, 5, Expect::Ok(1));
    dual_run(&mut dev, next(), "branch-taken", BRANCHDIR, 0, 20, Expect::Ok(2));
    dual_run(&mut dev, next(), "branch-edge", BRANCHDIR, 0, 10, Expect::Ok(2));

    // backward-branch loop: f(a) = a*(a+1)/2
    for a in [0u32, 1, 10, 50] {
        dual_run(&mut dev, next(), "sumloop", SUMLOOP, 0, a, Expect::Ok(a * (a + 1) / 2));
    }

    // --- window-pressure: deep recursion forcing overflow/underflow ---
    // call8 wraps the 64-reg file every 8 frames; depth 30/100 forces many
    // spill/reload round-trips — the key window-machinery check. f(d) = d.
    for d in [1u32, 7, 8, 9, 16, 17, 30, 100] {
        dual_run(&mut dev, next(), "rec8", REC8, 0, d, Expect::Ok(d));
    }
    // call12 (inc=3) recursion — a different rotation width. f(d) = d.
    for d in [1u32, 5, 16, 30, 60] {
        dual_run(&mut dev, next(), "rec12", REC12, 0, d, Expect::Ok(d));
    }

    // --- fault cases (crash classification must agree) ---
    dual_run(&mut dev, next(), "crash-ill", CRASH_ILL, 0, 0, Expect::Crash(TrapKind::Exception));
    dual_run(&mut dev, next(), "hang", HANG, 0, 0, Expect::Crash(TrapKind::Timeout));

    eprintln!("corpus_dual_run: all cases passed (device={})", dev.is_some());
}

// ---------------------------------------------------------------------------
// Emulator-only golden vectors that self-address via callx8 + l32r (absolute
// literals), so they cannot be position-independently run on the device. These
// exercise the callx8 + l32r-literal + builtin-call paths.
// ---------------------------------------------------------------------------

#[test]
fn gv2_callx8_builtin() {
    // Layout: [CALL_BLOB][pad to 4][MUL3]; patch the literal slot to mul3's
    // I-bus address. f() = builtin(42) = 126.
    let mut code = CALL_BLOB.to_vec();
    while !code.len().is_multiple_of(4) {
        code.push(0);
    }
    let mul3_off = code.len() as u32;
    code.extend_from_slice(MUL3);
    patch_u32(&mut code, 0, emu_entry(mul3_off));

    match emu_run(&code, 4, 0) {
        EmuOutcome::Ok(v) => assert_eq!(v, 126, "GV2 callx8 builtin(42) = 3*42"),
        other => panic!("GV2 unexpected: {other:?}"),
    }
}

#[test]
fn gv3a_callx8_recursion() {
    // Self literal at +0 → entry (+4). f(d) = d, via callx8 recursion.
    for d in [0u32, 1, 8, 17, 30, 100] {
        let mut code = REC_BLOB.to_vec();
        patch_u32(&mut code, 0, emu_entry(4));
        match emu_run(&code, 4, d) {
            EmuOutcome::Ok(v) => assert_eq!(v, d, "GV3a f({d}) = {d}"),
            other => panic!("GV3a d={d} unexpected: {other:?}"),
        }
    }
}

#[test]
fn gv3b_recursion_with_builtin_base() {
    // Self +0 → entry (+8); builtin +4 → mul3. Base case does builtin(7)=21,
    // each level adds 1: f(d) = d + 21.
    for d in [0u32, 1, 8, 17, 30, 100] {
        let mut code = RECB_BLOB.to_vec();
        while !code.len().is_multiple_of(4) {
            code.push(0);
        }
        let mul3_off = code.len() as u32;
        code.extend_from_slice(MUL3);
        patch_u32(&mut code, 0, emu_entry(8)); // self
        patch_u32(&mut code, 4, emu_entry(mul3_off)); // builtin
        match emu_run(&code, 8, d) {
            EmuOutcome::Ok(v) => assert_eq!(v, d + 21, "GV3b f({d}) = {d}+21"),
            other => panic!("GV3b d={d} unexpected: {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Trace hook smoke test: the text tracer produces a readable per-instruction +
// window-event log.
// ---------------------------------------------------------------------------

#[test]
fn trace_hook_produces_window_events() {
    use lp_xt_emu::TextTracer;
    let mut emu = Emulator::new();
    let mut tr = TextTracer::new();
    // rec8 to a depth that overflows (forces spill + reload lines).
    let out = emu.run_traced(REC8, 0, 12, &mut tr);
    assert_eq!(out, EmuOutcome::Ok(12));
    let text = tr.dump();
    assert!(text.contains("entry"), "trace should show ENTRY rotations");
    assert!(text.contains("retw"), "trace should show RETW rotations");
    assert!(text.contains("spill"), "deep recursion should spill: {text}");
    assert!(text.contains("reload"), "deep recursion should reload");
    eprintln!("--- sample trace (first 25 lines) ---");
    for line in text.lines().take(25) {
        eprintln!("{line}");
    }
}
