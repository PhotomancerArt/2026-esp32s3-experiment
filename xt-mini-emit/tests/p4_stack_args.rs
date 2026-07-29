//! P4 — stack-passed args, sret, multi-return (dual-run rig).
//!
//! Convention under test (esp-toolchain oracle: `fixtures/elf/call_conv.elf`,
//! disassembled with `xtensa-esp32s3-elf-objdump`; matched, not invented):
//!
//! - **Stack args**: the caller stores args `max_reg_args..` (7+ under CALL8)
//!   at the *bottom of its own frame*, `[SP + 4*(i - max_reg_args), …)`; the
//!   callee reads them at `[callee_SP + callee_frame + 4*(i - max_reg_args)]`
//!   (callee SP + ENTRY frame == caller SP). Oracle: `many` reads args 7/8 at
//!   `l32i a1, 32/36` under `entry a1, 32`; its caller stores 7/8 at
//!   `s32i a1, 0/4`. Identical region order to rv32's
//!   `[SP, SP + caller_arg_stack_size)`.
//! - **sret**: returns wider than `SRET_SCALAR_THRESHOLD` (= 2) words go
//!   through a caller-allocated buffer whose address is passed as the FIRST
//!   argument (callee `a2`); the callee stores through it and returns no
//!   register value. Oracle: `make_quad` writes `a2+0..12` and `retw`s.
//! - **Multi-return**: 2 words return direct in callee `a2, a3` -> caller
//!   `a10, a11` (`RET_REGS` / `CALL_RET_REGS`).
//!
//! Every program here is position-independent and dual-runs: emulator always,
//! plus the real ESP32-S3 when `XT_DEVICE_PORT` is set (single-threaded:
//! `-- --test-threads=1`).

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::Emulator;

use xt_mini_emit::{
    emit_program, emit_program_with, AluImmOp, AluOp, CallInc, Callee, EmitOut, MiniFunc,
    MiniProgram, MiniVInst, PReg,
};
use xt_runner_client::{RunOutcome as HwOutcome, Runner};

use MiniVInst::{
    AluRRI, AluRRR, BrIf, CallMulti, IConst32, IncomingStackArg, Label, Load32, Ret, RetMulti,
    SlotAddr, Store32,
};

fn p(n: u8) -> PReg {
    PReg(n)
}

fn func(slots: Vec<u32>, insts: Vec<MiniVInst>) -> MiniFunc {
    MiniFunc { slots, insts }
}

// ---------------------------------------------------------------------------
// Harness (device() pattern shared with tests/dual_run.rs)
// ---------------------------------------------------------------------------

fn emu_run(code: &[u8], entry_offset: u32, arg: u32) -> EmuOutcome {
    let mut emu = Emulator::new();
    emu.run(code, entry_offset, arg)
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

fn dual_run(
    device: &mut Option<Runner>,
    seq: &mut u32,
    name: &str,
    out: &EmitOut,
    arg: u32,
    expect: u32,
) {
    assert!(
        out.sym_slots.is_empty(),
        "[{name}] dual-run programs must be position-independent (no sym slots)"
    );
    match emu_run(&out.code, out.entry_offset, arg) {
        EmuOutcome::Ok(got) => {
            assert_eq!(got, expect, "[{name}] emu result mismatch (arg={arg})")
        }
        other => panic!("[{name}] emu outcome {other:?}, expected Ok({expect}) (arg={arg})"),
    }

    let Some(dev) = device.as_mut() else { return };
    *seq += 1;
    let hw = dev
        .load_exec(*seq, out.code.clone(), out.entry_offset, arg)
        .unwrap_or_else(|e| panic!("[{name}] device load_exec failed: {e}"));
    match hw {
        HwOutcome::Ok(h) => assert_eq!(h, expect, "[{name}] EMU vs HW diff (arg={arg})"),
        HwOutcome::Crash(r) => panic!("[{name}] device crashed (arg={arg}): {r:?}"),
    }
}

// ---------------------------------------------------------------------------
// 1. Eight-argument call (2 stack-passed) — the oracle's `many` shape.
// ---------------------------------------------------------------------------

/// Callee (func 1): weighted fold of all 8 args, `sum(arg_i << i)`.
/// Args 0..=5 arrive in a2..=a7; args 6/7 are read with IncomingStackArg.
/// Distinct weights make any arg-slot / stack-offset mixup change the result.
fn many8_callee() -> MiniFunc {
    let mut insts = vec![];
    // Shift each register arg by its weight in place (arg0 << 0 is a no-op).
    for i in 1..6u8 {
        insts.push(AluRRI {
            op: AluImmOp::Slli,
            dst: p(2 + i),
            src: p(2 + i),
            imm: i as i32,
        });
    }
    // Sum into a2.
    for i in 1..6u8 {
        insts.push(AluRRR {
            op: AluOp::Add,
            dst: p(2),
            src1: p(2),
            src2: p(2 + i),
        });
    }
    // Stack args 6 and 7 (a3 is dead by now).
    for (idx, w) in [(6u32, 6i32), (7, 7)] {
        insts.push(IncomingStackArg {
            dst: p(3),
            index: idx,
        });
        insts.push(AluRRI {
            op: AluImmOp::Slli,
            dst: p(3),
            src: p(3),
            imm: w,
        });
        insts.push(AluRRR {
            op: AluOp::Add,
            dst: p(2),
            src1: p(2),
            src2: p(3),
        });
    }
    insts.push(Ret { val: Some(p(2)) });
    func(vec![], insts)
}

/// f(a) = many8(a, a+1, a+2, a+3, a+4, a+5, a+1, a) — six distinct register
/// args plus two stack args that duplicate registers (only 6 program regs
/// exist), still offset-discriminating because the weights differ.
fn prog_many8() -> MiniProgram {
    let mut main = vec![];
    for i in 1..6u8 {
        main.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(2 + i),
            src: p(2),
            imm: i as i32,
        });
    }
    main.push(CallMulti {
        callee: Callee::Func(1),
        args: vec![p(2), p(3), p(4), p(5), p(6), p(7), p(3), p(2)],
        rets: vec![p(2)],
    });
    main.push(Ret { val: Some(p(2)) });
    MiniProgram {
        funcs: vec![func(vec![], main), many8_callee()],
    }
}

fn many8_expect(a: u32) -> u32 {
    let vals = [
        a,
        a.wrapping_add(1),
        a.wrapping_add(2),
        a.wrapping_add(3),
        a.wrapping_add(4),
        a.wrapping_add(5),
        a.wrapping_add(1), // stack arg 6 = p3
        a,                 // stack arg 7 = p2
    ];
    vals.iter()
        .enumerate()
        .fold(0u32, |acc, (i, &v)| acc.wrapping_add(v << i))
}

// ---------------------------------------------------------------------------
// 2. Two-word direct return (RET_REGS = a2,a3 callee -> a10,a11 caller).
// ---------------------------------------------------------------------------

/// Callee (func 1): (lo, hi) = (x + 1, x ^ 0x5A5A). The RetMulti vals are
/// [p3, p2] — hi is computed into a2 and lo into a3, so writing the return
/// pair (a2 = lo, a3 = hi) is a full swap: exercises the scratch-bounce path.
fn ret2_callee() -> MiniFunc {
    func(
        vec![],
        vec![
            // a3 = x + 1 (lo), a2 = x ^ 0x5A5A (hi).
            AluRRI {
                op: AluImmOp::Addi,
                dst: p(3),
                src: p(2),
                imm: 1,
            },
            AluRRI {
                op: AluImmOp::Xori,
                dst: p(2),
                src: p(2),
                imm: 0x5A5A,
            },
            RetMulti {
                vals: vec![p(3), p(2)],
            },
        ],
    )
}

/// f(a) = lo + 3*hi where (lo, hi) = ret2(a).
fn prog_ret2() -> MiniProgram {
    let main = func(
        vec![],
        vec![
            CallMulti {
                callee: Callee::Func(1),
                args: vec![p(2)],
                rets: vec![p(3), p(4)],
            },
            IConst32 { dst: p(5), val: 3 },
            AluRRR {
                op: AluOp::Mul,
                dst: p(4),
                src1: p(4),
                src2: p(5),
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(2),
                src1: p(3),
                src2: p(4),
            },
            Ret { val: Some(p(2)) },
        ],
    );
    MiniProgram {
        funcs: vec![main, ret2_callee()],
    }
}

fn ret2_expect(a: u32) -> u32 {
    let (lo, hi) = (a.wrapping_add(1), a ^ 0x5A5A);
    lo.wrapping_add(hi.wrapping_mul(3))
}

// ---------------------------------------------------------------------------
// 3. sret: 4-word (vec4-shaped) return through a caller-allocated buffer.
// ---------------------------------------------------------------------------

/// Callee (func 1): quad filler. Args: a2 = buffer pointer (the sret slot,
/// FIRST arg — the oracle's `make_quad` shape), a3 = x. Writes
/// [x+1, x*3, x ^ 0xFF00FF, x<<9 | x>>23] and returns nothing.
fn quad_callee() -> MiniFunc {
    func(
        vec![],
        vec![
            AluRRI {
                op: AluImmOp::Addi,
                dst: p(4),
                src: p(3),
                imm: 1,
            },
            Store32 {
                src: p(4),
                base: p(2),
                offset: 0,
            },
            IConst32 { dst: p(5), val: 3 },
            AluRRR {
                op: AluOp::Mul,
                dst: p(4),
                src1: p(3),
                src2: p(5),
            },
            Store32 {
                src: p(4),
                base: p(2),
                offset: 4,
            },
            AluRRI {
                op: AluImmOp::Xori,
                dst: p(4),
                src: p(3),
                imm: 0x00FF_00FF,
            },
            Store32 {
                src: p(4),
                base: p(2),
                offset: 8,
            },
            // rotate_left(x, 9) = (x << 9) | (x >> 23)
            AluRRI {
                op: AluImmOp::Slli,
                dst: p(4),
                src: p(3),
                imm: 9,
            },
            AluRRI {
                op: AluImmOp::SrliU,
                dst: p(5),
                src: p(3),
                imm: 23,
            },
            AluRRR {
                op: AluOp::Or,
                dst: p(4),
                src1: p(4),
                src2: p(5),
            },
            Store32 {
                src: p(4),
                base: p(2),
                offset: 12,
            },
            Ret { val: None },
        ],
    )
}

/// f(a): q = quad(a) via sret; fold = ((q0 + q1) ^ q2) + q3.
fn prog_sret4() -> MiniProgram {
    let main = func(
        vec![16],
        vec![
            SlotAddr { dst: p(3), slot: 0 },
            CallMulti {
                callee: Callee::Func(1),
                args: vec![p(3), p(2)],
                rets: vec![],
            },
            SlotAddr { dst: p(3), slot: 0 },
            Load32 {
                dst: p(4),
                base: p(3),
                offset: 0,
            },
            Load32 {
                dst: p(5),
                base: p(3),
                offset: 4,
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(4),
                src1: p(4),
                src2: p(5),
            },
            Load32 {
                dst: p(5),
                base: p(3),
                offset: 8,
            },
            AluRRR {
                op: AluOp::Xor,
                dst: p(4),
                src1: p(4),
                src2: p(5),
            },
            Load32 {
                dst: p(5),
                base: p(3),
                offset: 12,
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(4),
                src1: p(4),
                src2: p(5),
            },
            Ret { val: Some(p(4)) },
        ],
    );
    MiniProgram {
        funcs: vec![main, quad_callee()],
    }
}

fn quad_expect(x: u32) -> [u32; 4] {
    [
        x.wrapping_add(1),
        x.wrapping_mul(3),
        x ^ 0x00FF_00FF,
        x.rotate_left(9),
    ]
}

fn sret4_expect(a: u32) -> u32 {
    let q = quad_expect(a);
    (q[0].wrapping_add(q[1]) ^ q[2]).wrapping_add(q[3])
}

// ---------------------------------------------------------------------------
// 4. Deep recursion: stack args + sret buffers live while windows spill.
// ---------------------------------------------------------------------------

/// g(d) (func 1), returning a pair:
///   d == 0            -> (1, 2)
///   d != 0:
///     save d to a slot
///     (lo, hi) = g(d - 1)                      // 2-word return at depth
///     m = many8(d, lo, hi, d, lo, hi, d, lo)   // 8-arg call (2 stack args)
///     quad(buf, d) via sret                    // buffer in this frame
///     reload = saved d                         // slot integrity check
///     lo' = lo + m + q0 + q3 + (reload - d) + 1
///     hi' = hi + q1 + q2 + d
///   -> (lo', hi')
///
/// Every frame keeps a live sret buffer and saved-slot word across the
/// recursive call, and stages stack args below them — past depth ~6 the
/// window file wraps, so the save-area spills happen *around* both regions.
/// main (func 0) folds the pair to a single word for the harness.
fn deep_g() -> MiniFunc {
    let mut insts = vec![
        BrIf {
            cond: p(2),
            target: 0,
            invert: false,
        },
        IConst32 { dst: p(3), val: 1 },
        IConst32 { dst: p(4), val: 2 },
        RetMulti {
            vals: vec![p(3), p(4)],
        },
        Label(0),
        // slot 1: saved d.
        SlotAddr { dst: p(3), slot: 1 },
        Store32 {
            src: p(2),
            base: p(3),
            offset: 0,
        },
        // (lo, hi) = g(d - 1) -> p4, p5.
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(3),
            src: p(2),
            imm: -1,
        },
        CallMulti {
            callee: Callee::Func(1),
            args: vec![p(3)],
            rets: vec![p(4), p(5)],
        },
        // m = many8(d, lo, hi, d, lo, hi, d, lo) -> p6.
        CallMulti {
            callee: Callee::Func(2),
            args: vec![p(2), p(4), p(5), p(2), p(4), p(5), p(2), p(4)],
            rets: vec![p(6)],
        },
        // quad(buf, d) via sret into slot 0.
        SlotAddr { dst: p(3), slot: 0 },
        CallMulti {
            callee: Callee::Func(3),
            args: vec![p(3), p(2)],
            rets: vec![],
        },
        // lo' = lo + m + q0 + q3 + (reload - d) + 1.
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(6),
        },
        SlotAddr { dst: p(3), slot: 0 },
        Load32 {
            dst: p(6),
            base: p(3),
            offset: 0,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(6),
        },
        Load32 {
            dst: p(6),
            base: p(3),
            offset: 12,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(6),
        },
        SlotAddr { dst: p(7), slot: 1 },
        Load32 {
            dst: p(7),
            base: p(7),
            offset: 0,
        },
        AluRRR {
            op: AluOp::Sub,
            dst: p(7),
            src1: p(7),
            src2: p(2),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(7),
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(4),
            src: p(4),
            imm: 1,
        },
        // hi' = hi + q1 + q2 + d.
        Load32 {
            dst: p(6),
            base: p(3),
            offset: 4,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(5),
            src1: p(5),
            src2: p(6),
        },
        Load32 {
            dst: p(6),
            base: p(3),
            offset: 8,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(5),
            src1: p(5),
            src2: p(6),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(5),
            src1: p(5),
            src2: p(2),
        },
        RetMulti {
            vals: vec![p(4), p(5)],
        },
    ];
    // (unreachable) keep label namespace tidy
    insts.push(Ret { val: None });
    func(vec![16, 4], insts)
}

fn prog_deep() -> MiniProgram {
    // main: (lo, hi) = g(d); return lo + hi * 0x101.
    let main = func(
        vec![],
        vec![
            CallMulti {
                callee: Callee::Func(1),
                args: vec![p(2)],
                rets: vec![p(3), p(4)],
            },
            IConst32 {
                dst: p(5),
                val: 0x101,
            },
            AluRRR {
                op: AluOp::Mul,
                dst: p(4),
                src1: p(4),
                src2: p(5),
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(2),
                src1: p(3),
                src2: p(4),
            },
            Ret { val: Some(p(2)) },
        ],
    );
    MiniProgram {
        funcs: vec![main, deep_g(), many8_callee(), quad_callee()],
    }
}

fn deep_g_expect(d: u32) -> (u32, u32) {
    if d == 0 {
        return (1, 2);
    }
    let (lo, hi) = deep_g_expect(d - 1);
    let vals = [d, lo, hi, d, lo, hi, d, lo];
    let m = vals
        .iter()
        .enumerate()
        .fold(0u32, |acc, (i, &v)| acc.wrapping_add(v << i));
    let q = quad_expect(d);
    let lo2 = lo
        .wrapping_add(m)
        .wrapping_add(q[0])
        .wrapping_add(q[3])
        .wrapping_add(1);
    let hi2 = hi.wrapping_add(q[1]).wrapping_add(q[2]).wrapping_add(d);
    (lo2, hi2)
}

fn deep_expect(d: u32) -> u32 {
    let (lo, hi) = deep_g_expect(d);
    lo.wrapping_add(hi.wrapping_mul(0x101))
}

// ---------------------------------------------------------------------------
// The single dual-run test (device cases must not run concurrently).
// ---------------------------------------------------------------------------

#[test]
fn p4_corpus_dual_run() {
    let mut dev = device();
    let mut seq = 7000u32;
    let args = [0u32, 1, 7, 42, 1000, 0xFFFF_FFFB, 0x8000_0000];

    let many8 = emit_program(&prog_many8());
    for a in args {
        dual_run(&mut dev, &mut seq, "p4-many8", &many8, a, many8_expect(a));
    }

    let ret2 = emit_program(&prog_ret2());
    for a in args {
        dual_run(&mut dev, &mut seq, "p4-ret2", &ret2, a, ret2_expect(a));
    }

    let sret4 = emit_program(&prog_sret4());
    for a in args {
        dual_run(&mut dev, &mut seq, "p4-sret4", &sret4, a, sret4_expect(a));
    }

    // Depths straddling the measured window-overflow onset (first spill at
    // depth 6 under CALL8) and the 64-register file wrap: stack args and
    // sret buffers stay correct while ancestor frames spill/reload.
    let deep = emit_program(&prog_deep());
    for d in [0u32, 1, 3, 5, 6, 7, 8, 12, 17, 30] {
        dual_run(&mut dev, &mut seq, "p4-deep", &deep, d, deep_expect(d));
    }

    eprintln!(
        "p4_corpus_dual_run: all cases passed (device={})",
        dev.is_some()
    );
}

// ---------------------------------------------------------------------------
// Emulator-only: the deep case must actually spill (not merely be correct).
// ---------------------------------------------------------------------------

#[test]
fn p4_deep_forces_window_traps() {
    use lp_xt_emu::TextTracer;
    let out = emit_program(&prog_deep());
    let mut emu = Emulator::new();
    let mut tr = TextTracer::new();
    let res = emu.run_traced(&out.code, out.entry_offset, 12, &mut tr);
    assert_eq!(res, EmuOutcome::Ok(deep_expect(12)));
    let text = tr.dump();
    assert!(text.contains("spill"), "depth-12 p4 recursion must spill");
    assert!(text.contains("reload"), "depth-12 p4 recursion must reload");
}

// ---------------------------------------------------------------------------
// Threshold behavior: 2 direct return words are the ceiling; 3 must refuse
// (sret buffer is the only lowering), matching SRET_SCALAR_THRESHOLD = 2.
// ---------------------------------------------------------------------------

#[test]
fn p4_three_direct_return_words_refused() {
    let prog = MiniProgram {
        funcs: vec![func(
            vec![],
            vec![RetMulti {
                vals: vec![p(2), p(3), p(4)],
            }],
        )],
    };
    assert!(
        std::panic::catch_unwind(|| emit_program(&prog)).is_err(),
        "RetMulti with 3 words must refuse to emit (threshold is 2)"
    );

    let prog = MiniProgram {
        funcs: vec![
            func(
                vec![],
                vec![
                    CallMulti {
                        callee: Callee::Func(1),
                        args: vec![p(2)],
                        rets: vec![p(3), p(4), p(5)],
                    },
                    Ret { val: Some(p(3)) },
                ],
            ),
            func(vec![], vec![Ret { val: Some(p(2)) }]),
        ],
    };
    assert!(
        std::panic::catch_unwind(|| emit_program(&prog)).is_err(),
        "CallMulti with 3 ret words must refuse to emit (threshold is 2)"
    );
}

// ---------------------------------------------------------------------------
// Emulator-only: CALL12's 2-register-arg ceiling is escapable via stack args
// (CallMulti); the P1 finding was that *plain* Call cannot stage a 3rd
// register arg — with the P4 outgoing-arg area, arg 2 goes to the stack.
// ---------------------------------------------------------------------------

#[test]
fn p4_call12_three_args_via_stack_emu() {
    // f(a) = h(a, a+1, a+2) with h = w0 + 2*w1 + 4*w2; under CALL12 args 0/1
    // ride a14/a15 and arg 2 is stack-passed.
    let main = func(
        vec![],
        vec![
            AluRRI {
                op: AluImmOp::Addi,
                dst: p(3),
                src: p(2),
                imm: 1,
            },
            AluRRI {
                op: AluImmOp::Addi,
                dst: p(4),
                src: p(2),
                imm: 2,
            },
            CallMulti {
                callee: Callee::Func(1),
                args: vec![p(2), p(3), p(4)],
                rets: vec![p(2)],
            },
            Ret { val: Some(p(2)) },
        ],
    );
    let callee = func(
        vec![],
        vec![
            AluRRI {
                op: AluImmOp::Slli,
                dst: p(3),
                src: p(3),
                imm: 1,
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(2),
                src1: p(2),
                src2: p(3),
            },
            IncomingStackArg {
                dst: p(3),
                index: 2,
            },
            AluRRI {
                op: AluImmOp::Slli,
                dst: p(3),
                src: p(3),
                imm: 2,
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(2),
                src1: p(2),
                src2: p(3),
            },
            Ret { val: Some(p(2)) },
        ],
    );
    let out = emit_program_with(
        &MiniProgram {
            funcs: vec![main, callee],
        },
        CallInc::Call12,
    );
    for a in [0u32, 1, 41, 1000] {
        let expect = a
            .wrapping_add(a.wrapping_add(1) << 1)
            .wrapping_add(a.wrapping_add(2) << 2);
        match emu_run(&out.code, out.entry_offset, a) {
            EmuOutcome::Ok(v) => assert_eq!(v, expect, "call12-stack f({a})"),
            other => panic!("call12-stack a={a} unexpected: {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// The IsaTarget-shaped rules P4 exists to pin (see README "Argument passing
// and returns"): values the monorepo's `isa/xt` arm will return.
// ---------------------------------------------------------------------------

#[test]
fn p4_isa_target_shaped_rules() {
    use xt_mini_emit::{abi, gpr};

    // call_arg_reg_count() = 6; call_arg_reg_hw(i) = ARG_REGS[i] (a2..=a7).
    assert_eq!(gpr::ARG_REGS.len(), 6);
    assert_eq!(CallInc::Call8.max_reg_args(), gpr::ARG_REGS.len());

    // direct_ret_reg_count() = 2; direct_ret_reg_hw = RET_REGS (a2, a3).
    assert_eq!(gpr::RET_REGS.len(), 2);

    // sret_uses_buffer_for(n) == n > SRET_SCALAR_THRESHOLD (= 2): validated
    // above — 2-word returns emit direct (p4-ret2), 3 refuse, 4 go through
    // the caller buffer (p4-sret4).
    assert_eq!(abi::SRET_SCALAR_THRESHOLD, 2);

    // lpir_call_stack_args_start(callee_uses_sret, caller_passes_sret_ptr):
    // same formula as rv32 with ARG_REGS.len() = 6 — the sret pointer, when
    // injected by the emitter, occupies ARG_REGS[0] and leaves 5 registers
    // for explicit operands.
    let stack_args_start = |callee_uses_sret: bool, caller_passes_sret_ptr: bool| -> usize {
        if callee_uses_sret && !caller_passes_sret_ptr {
            gpr::ARG_REGS.len() - 1
        } else {
            gpr::ARG_REGS.len()
        }
    };
    assert_eq!(stack_args_start(false, false), 6);
    assert_eq!(stack_args_start(true, false), 5);
    assert_eq!(stack_args_start(true, true), 6);
}
