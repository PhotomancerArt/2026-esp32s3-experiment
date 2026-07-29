//! Conformance corpus, N-run (P5).
//!
//! Every case runs on `lp-xt-emu` under **every** board profile (S3 and
//! classic memory maps) against its known answer. Each attached board —
//! `XT_PORT_ESP32S3` (alias `XT_DEVICE_PORT`) and/or `XT_PORT_ESP32` — also
//! runs every position-independent case, and value/crash-class must agree
//! across all worlds (`xt-testkit` is the shared harness).
//!
//! Hardware tests share the boards — run single-threaded:
//!   XT_PORT_ESP32S3=... XT_PORT_ESP32=... cargo test -p lp-xt-emu -- --test-threads=1 --nocapture
//!
//! All payload bytes are objdump-derived (see the scratch `.s` sources and
//! FINDINGS.md), never hand-recalled.

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::{Emulator, TrapKind};

use xt_testkit::{known_profiles, Expect, Harness};

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
// Harness (shared: xt-testkit N-runs each case on every profile + board)
// ---------------------------------------------------------------------------

/// Patch a little-endian u32 into `code` at `off`.
fn patch_u32(code: &mut [u8], off: usize, val: u32) {
    code[off..off + 4].copy_from_slice(&val.to_le_bytes());
}

// ---------------------------------------------------------------------------
// The one #[test] entry point: runs the whole corpus sequentially so a single
// board is never driven concurrently.
// ---------------------------------------------------------------------------

#[test]
fn corpus_dual_run() {
    let mut h = Harness::from_env(1);

    // --- basic value-returning cases (dual-run capable) ---
    h.nrun_expect("stub42", STUB42, 0, 0, Expect::Ok(42));
    h.nrun_expect("stub42-argignored", STUB42, 0, 999, Expect::Ok(42));
    h.nrun_expect("identity", IDENTITY, 0, 1234, Expect::Ok(1234));
    h.nrun_expect("identity-0", IDENTITY, 0, 0, Expect::Ok(0));

    // arithmetic: f(a) = 3a + 13
    for a in [0u32, 1, 7, 1000] {
        h.nrun_expect("arith", ARITH, 0, a, Expect::Ok(3 * a + 13));
    }

    // load/store round-trip: f(a) = a + 100
    for a in [0u32, 5, 42, 100000] {
        h.nrun_expect("loadstore", LOADSTORE, 0, a, Expect::Ok(a + 100));
    }

    // branches both directions: f(a) = if a>=10 {2} else {1}
    h.nrun_expect("branch-nottaken", BRANCHDIR, 0, 5, Expect::Ok(1));
    h.nrun_expect("branch-taken", BRANCHDIR, 0, 20, Expect::Ok(2));
    h.nrun_expect("branch-edge", BRANCHDIR, 0, 10, Expect::Ok(2));

    // backward-branch loop: f(a) = a*(a+1)/2
    for a in [0u32, 1, 10, 50] {
        h.nrun_expect("sumloop", SUMLOOP, 0, a, Expect::Ok(a * (a + 1) / 2));
    }

    // --- window-pressure: deep recursion forcing overflow/underflow ---
    // call8 wraps the 64-reg file every 8 frames; depth 30/100 forces many
    // spill/reload round-trips — the key window-machinery check. f(d) = d.
    for d in [1u32, 7, 8, 9, 16, 17, 30, 100] {
        h.nrun_expect("rec8", REC8, 0, d, Expect::Ok(d));
    }
    // call12 (inc=3) recursion — a different rotation width. f(d) = d.
    for d in [1u32, 5, 16, 30, 60] {
        h.nrun_expect("rec12", REC12, 0, d, Expect::Ok(d));
    }

    // --- fault cases (crash classification must agree) ---
    h.nrun_expect("crash-ill", CRASH_ILL, 0, 0, Expect::Crash(TrapKind::Exception));
    h.nrun_expect("hang", HANG, 0, 0, Expect::Crash(TrapKind::Timeout));

    eprintln!(
        "corpus_dual_run: all cases passed (boards={})",
        h.boards.len()
    );
}

// ---------------------------------------------------------------------------
// Emulator-only golden vectors that self-address via callx8 + l32r (absolute
// literals), so they cannot be position-independently run on the device. These
// exercise the callx8 + l32r-literal + builtin-call paths — linked and run per
// board profile (the absolute addresses come from each profile's
// `code_ibus_base()`, never a hardcoded map).
// ---------------------------------------------------------------------------

/// Run a self-addressing blob on the emulator under `profile`, with `link`
/// patching the absolute literals for that profile's I-bus code base.
fn emu_run_linked(
    profile: lp_xt_emu::BoardProfile,
    code: &mut [u8],
    entry_offset: u32,
    arg: u32,
    link: impl Fn(&mut [u8], u32),
) -> EmuOutcome {
    link(code, profile.code_ibus_base());
    let mut emu = Emulator::with_profile(profile);
    emu.run(code, entry_offset, arg)
}

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

    for (chip, profile) in known_profiles() {
        let mut blob = code.clone();
        let got = emu_run_linked(profile, &mut blob, 4, 0, |c, base| {
            patch_u32(c, 0, base + mul3_off);
        });
        match got {
            EmuOutcome::Ok(v) => assert_eq!(v, 126, "GV2 [{chip}] callx8 builtin(42) = 3*42"),
            other => panic!("GV2 [{chip}] unexpected: {other:?}"),
        }
    }
}

#[test]
fn gv3a_callx8_recursion() {
    // Self literal at +0 → entry (+4). f(d) = d, via callx8 recursion.
    for (chip, profile) in known_profiles() {
        for d in [0u32, 1, 8, 17, 30, 100] {
            let mut code = REC_BLOB.to_vec();
            let got = emu_run_linked(profile, &mut code, 4, d, |c, base| {
                patch_u32(c, 0, base + 4);
            });
            match got {
                EmuOutcome::Ok(v) => assert_eq!(v, d, "GV3a [{chip}] f({d}) = {d}"),
                other => panic!("GV3a [{chip}] d={d} unexpected: {other:?}"),
            }
        }
    }
}

#[test]
fn gv3b_recursion_with_builtin_base() {
    // Self +0 → entry (+8); builtin +4 → mul3. Base case does builtin(7)=21,
    // each level adds 1: f(d) = d + 21.
    let mut blob = RECB_BLOB.to_vec();
    while !blob.len().is_multiple_of(4) {
        blob.push(0);
    }
    let mul3_off = blob.len() as u32;
    blob.extend_from_slice(MUL3);

    for (chip, profile) in known_profiles() {
        for d in [0u32, 1, 8, 17, 30, 100] {
            let mut code = blob.clone();
            let got = emu_run_linked(profile, &mut code, 8, d, |c, base| {
                patch_u32(c, 0, base + 8); // self
                patch_u32(c, 4, base + mul3_off); // builtin
            });
            match got {
                EmuOutcome::Ok(v) => assert_eq!(v, d + 21, "GV3b [{chip}] f({d}) = {d}+21"),
                other => panic!("GV3b [{chip}] d={d} unexpected: {other:?}"),
            }
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
