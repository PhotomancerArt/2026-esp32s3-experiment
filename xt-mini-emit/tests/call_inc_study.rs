//! P1 — CALL-increment policy study (measurement rig).
//!
//! Measures, for CALL4/CALL8/CALL12, on the emulator (every board profile)
//! and on every attached board (`XT_PORT_ESP32S3` / `XT_PORT_ESP32`):
//!
//! 1. **Register-argument capacity** — how many arguments actually pass in
//!    registers (and that programs beyond the capacity refuse to emit).
//! 2. **Preserved temporaries** — which caller registers empirically survive
//!    a call into a callee that actively clobbers its whole window.
//! 3. **Window-overflow onset** — recursion at depths 1..=40, counting
//!    spill/reload events via `lp-xt-emu`'s `TextTracer` (the emulator's
//!    window machinery is silicon-validated, see FINDINGS.md) and verifying
//!    results on hardware across the onset.
//!
//! Results + recommendation: `xt-mini-emit/docs/call-inc-study.md`.
//!
//! Hardware tests share the boards — run single-threaded:
//!   XT_PORT_ESP32S3=... XT_PORT_ESP32=... cargo test -p xt-mini-emit -- --test-threads=1 --nocapture

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::{Emulator, TextTracer};
use lp_xt_inst::{encode, AluRrr, Inst, NullaryOp, Reg};

use xt_mini_emit::{
    emit_program_with, AluImmOp, AluOp, CallInc, Callee, EmitOut, IcmpCond, MiniFunc, MiniProgram,
    MiniVInst, PReg,
};
use xt_testkit::Harness;

use MiniVInst::{AluRRI, AluRRR, BrIf, Call, IConst32, IcmpImm, Label, Ret};

const INCS: [CallInc; 3] = [CallInc::Call4, CallInc::Call8, CallInc::Call12];

fn p(n: u8) -> PReg {
    PReg(n)
}

fn func(slots: Vec<u32>, insts: Vec<MiniVInst>) -> MiniFunc {
    MiniFunc { slots, insts }
}

// ---------------------------------------------------------------------------
// Harness (shared: xt-testkit N-runs each case on every profile + board;
// `Harness::measure` = every world must agree on the value, which is the
// *measurement*; `Harness::run_all` = per-world values for predicate-only
// comparisons)
// ---------------------------------------------------------------------------

fn measure_out(h: &mut Harness, name: &str, out: &EmitOut, arg: u32) -> u32 {
    assert!(out.sym_slots.is_empty(), "[{name}] must be position-independent");
    h.measure(name, &out.code, out.entry_offset, arg)
}

fn inc_name(inc: CallInc) -> &'static str {
    match inc {
        CallInc::Call4 => "call4",
        CallInc::Call8 => "call8",
        CallInc::Call12 => "call12",
    }
}

// ---------------------------------------------------------------------------
// 1. Register-argument capacity
// ---------------------------------------------------------------------------

/// main: derives `n` argument values from the input (`arg_i = a + i`), calls
/// the callee with all of them, returns its result. callee: order-sensitive
/// shift-accumulate fold of its `a2..a(1+n)` (no free register needed).
fn prog_nargs(n: usize) -> MiniProgram {
    let mut main = vec![];
    for i in 1..n {
        main.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(2 + i as u8),
            src: p(2),
            imm: i as i32,
        });
    }
    main.push(Call {
        callee: Callee::Func(1),
        args: (0..n).map(|i| p(2 + i as u8)).collect(),
        ret: Some(p(2)),
    });
    main.push(Ret { val: Some(p(2)) });

    let mut callee = vec![];
    for i in 1..n {
        callee.push(AluRRI {
            op: AluImmOp::Slli,
            dst: p(2),
            src: p(2),
            imm: 1,
        });
        callee.push(AluRRR {
            op: AluOp::Add,
            dst: p(2),
            src1: p(2),
            src2: p(2 + i as u8),
        });
    }
    callee.push(Ret { val: Some(p(2)) });

    MiniProgram {
        funcs: vec![func(vec![], main), func(vec![], callee)],
    }
}

fn nargs_expect(a: u32, n: usize) -> u32 {
    let mut acc = a;
    for i in 1..n {
        acc = (acc << 1).wrapping_add(a.wrapping_add(i as u32));
    }
    acc
}

#[test]
fn arg_capacity() {
    let mut h = Harness::from_env(5000);
    let args = [0u32, 5, 1000, 0xFFFF_FFF0];

    for inc in INCS {
        let cap = inc.max_reg_args();
        for n in 1..=cap {
            let out = emit_program_with(&prog_nargs(n), inc);
            for a in args {
                let name = format!("nargs-{}-{n}", inc_name(inc));
                let got = measure_out(&mut h, &name, &out, a);
                assert_eq!(got, nargs_expect(a, n), "[{name}] wrong fold (a={a})");
            }
        }
        // One past the capacity must refuse to emit — that IS the finding
        // (the mini ABI has no stack args; P4 adds them).
        let over = cap + 1;
        let refused =
            std::panic::catch_unwind(|| emit_program_with(&prog_nargs(over), inc)).is_err();
        assert!(
            refused,
            "{} must refuse to emit {over} register args",
            inc_name(inc)
        );
        eprintln!(
            "MEASURE arg-capacity inc={} max_reg_args={cap} ({over} args refused at emit)",
            inc_name(inc)
        );
    }
}

// ---------------------------------------------------------------------------
// 2. Preserved temporaries
// ---------------------------------------------------------------------------

/// Emitter-level probe for a program register `r` in a2..=a7: plant a known
/// constant in `a{r}`, call a callee that clobbers its whole program range
/// (its a2..=a7), then return 1 iff `a{r}` still holds the constant.
fn prog_probe_low(r: u8) -> MiniProgram {
    let k = 0x500 + r as i32;
    let d = if r == 2 { 3 } else { 2 };
    let main = vec![
        IConst32 { dst: p(r), val: k },
        Call {
            callee: Callee::Func(1),
            args: vec![],
            ret: None,
        },
        IcmpImm {
            dst: p(d),
            src: p(r),
            imm: k,
            cond: IcmpCond::Eq,
        },
        Ret { val: Some(p(d)) },
    ];
    let clobber = (2u8..=7)
        .map(|i| IConst32 {
            dst: p(i),
            val: 0xC0 + i as i32,
        })
        .chain([Ret { val: None }])
        .collect();
    MiniProgram {
        funcs: vec![func(vec![], main), func(vec![], clobber)],
    }
}

/// Raw probe for a register `r` in a8..=a15 (outside the MiniVInst program
/// range, so built directly from `lp_xt_inst::encode` — no hand-encoded
/// bytes). Layout: clobber stub at offset 0, main after it (entry_offset).
///
/// stub: `entry a1,48; movi a2..a15 <junk>; retw` — actively writes every
/// non-a0/a1 register of its own window, so any caller register mapped into
/// the callee window is genuinely overwritten.
/// main: `entry a1,48; movi a{r},K; call<inc> stub; mov a2,a{r}; retw`.
/// Returns (code, entry_offset, planted K).
fn raw_probe(inc: CallInc, r: u8) -> (Vec<u8>, u32, u32) {
    let k = 1500 + 7 * r as i32; // movi range, distinct from stub junk
    let mut code: Vec<u8> = Vec::new();
    // Stub at offset 0.
    code.extend(encode(&Inst::Entry(Reg::new(1), 48)));
    for i in 2u8..=15 {
        code.extend(encode(&Inst::Movi(Reg::new(i), 100 + i as i32)));
    }
    code.extend(encode(&Inst::Nullary(NullaryOp::Retw)));
    assert_eq!(code.len() % 4, 0, "stub must leave main 4-aligned");
    let entry = code.len() as u32;
    // Main.
    code.extend(encode(&Inst::Entry(Reg::new(1), 48)));
    code.extend(encode(&Inst::Movi(Reg::new(r), k)));
    // call<inc> back to the stub: target = (PC & !3) + (off << 2) + 4.
    let pc = code.len() as i64;
    let off = (0 - (pc & !3) - 4) >> 2;
    assert_eq!((pc & !3) + (off << 2) + 4, 0, "call offset must be exact");
    code.extend(encode(&Inst::Call(inc.call_op(), off as i32)));
    // mov a2, a{r} (wide or-form, as the emitter emits).
    code.extend(encode(&Inst::Rrr(AluRrr::Or, Reg::new(2), Reg::new(r), Reg::new(r))));
    code.extend(encode(&Inst::Nullary(NullaryOp::Retw)));
    (code, entry, k as u32)
}

#[test]
fn preserved_temporaries() {
    let mut h = Harness::from_env(6000);

    for inc in INCS {
        let mut survived: Vec<u8> = Vec::new();
        // a2..=a7 through the emitter.
        for r in 2u8..=7 {
            let out = emit_program_with(&prog_probe_low(r), inc);
            let name = format!("probe-{}-a{r}", inc_name(inc));
            let got = measure_out(&mut h, &name, &out, 0);
            assert!(got <= 1, "[{name}] probe must return 0/1, got {got}");
            if got == 1 {
                survived.push(r);
            }
        }
        // a8..=a15 through raw (encode-built) programs. The raw post-call
        // value of a clobbered register can be position-dependent (e.g. under
        // CALL8, a8 comes back holding the mangled return address — top bits
        // 0b10 = CALLINC 2 — whose low bits differ by load address, and so by
        // world), so the survived/clobbered *predicate* is what every world
        // must agree on, not the raw value.
        for r in 8u8..=15 {
            let (code, entry, k) = raw_probe(inc, r);
            let name = format!("rawprobe-{}-a{r}", inc_name(inc));
            let results = h.run_all(&name, &code, entry, 0);
            let alive = results[0].1 == k;
            for (world, v) in &results[1..] {
                assert_eq!(
                    *v == k,
                    alive,
                    "[{name}] survival disagree: {}={:#010x} vs {world}={v:#010x} (k={k:#x})",
                    results[0].0, results[0].1
                );
            }
            if alive {
                survived.push(r);
            }
        }
        // The empirical rule: caller a{j} survives iff j < 4 * units (it sits
        // below the callee's rotated window). a0/a1 (RA/SP) are excluded.
        let expect: Vec<u8> = (2u8..=15).filter(|&j| j < 4 * inc.units()).collect();
        eprintln!(
            "MEASURE preserved inc={} survived={:?} (of a2..a15, callee clobbering its full window)",
            inc_name(inc),
            survived
        );
        assert_eq!(
            survived, expect,
            "{}: measured preserved set diverges from the window arithmetic",
            inc_name(inc)
        );
    }
}

// ---------------------------------------------------------------------------
// 3. Window-overflow onset (recursion depth sweep)
// ---------------------------------------------------------------------------

/// Self-recursion via CALLn: f(d) = d (same shape as dual_run's
/// prog_recursion; no register is live across the call except the emitter's
/// return plumbing, so it is emittable under every increment).
fn prog_recursion() -> MiniProgram {
    MiniProgram {
        funcs: vec![func(
            vec![],
            vec![
                BrIf {
                    cond: p(2),
                    target: 0,
                    invert: false,
                },
                IConst32 { dst: p(3), val: 0 },
                Ret { val: Some(p(3)) },
                Label(0),
                AluRRI {
                    op: AluImmOp::Addi,
                    dst: p(3),
                    src: p(2),
                    imm: -1,
                },
                Call {
                    callee: Callee::Func(0),
                    args: vec![p(3)],
                    ret: Some(p(4)),
                },
                AluRRI {
                    op: AluImmOp::Addi,
                    dst: p(5),
                    src: p(4),
                    imm: 1,
                },
                Ret { val: Some(p(5)) },
            ],
        )],
    }
}

#[test]
fn window_overflow_onset() {
    let mut h = Harness::from_env(7000);

    for inc in INCS {
        let out = emit_program_with(&prog_recursion(), inc);
        let mut onset = None;
        for d in 1u32..=40 {
            let mut emu = Emulator::new();
            let mut tr = TextTracer::new();
            let res = emu.run_traced(&out.code, out.entry_offset, d, &mut tr);
            assert_eq!(
                res,
                EmuOutcome::Ok(d),
                "recursion-{} emu f({d})",
                inc_name(inc)
            );
            let spills = tr.lines.iter().filter(|l| l.contains("spill")).count();
            let reloads = tr.lines.iter().filter(|l| l.contains("reload")).count();
            assert_eq!(spills, reloads, "every spilled frame must reload");
            // Spill *cost*: registers moved per event ("spill frame@baseN
            // (M regs) to ..."), summed — CALLn spills 4·n/4 regs per frame.
            let spill_regs: u32 = tr
                .lines
                .iter()
                .filter(|l| l.contains("spill"))
                .map(|l| {
                    let inner = l.split('(').nth(1).and_then(|s| s.split(' ').next());
                    inner.and_then(|s| s.parse::<u32>().ok()).unwrap_or(0)
                })
                .sum();
            if spills > 0 && onset.is_none() {
                onset = Some(d);
            }
            eprintln!(
                "MEASURE overflow inc={} depth={d} spills={spills} reloads={reloads} spilled_regs={spill_regs}",
                inc_name(inc)
            );
            // Hardware: verify the same program+depth computes correctly
            // across the onset (trap counts come from the silicon-validated
            // emulator; the device has no tracer).
            let name = format!("recursion-{}-d{d}", inc_name(inc));
            let got = measure_out(&mut h, &name, &out, d);
            assert_eq!(got, d, "[{name}] f(d) != d");
        }
        let onset = onset.expect("depth 40 must overflow under every increment");
        eprintln!("MEASURE overflow-onset inc={} first_spill_depth={onset}", inc_name(inc));
    }
}
