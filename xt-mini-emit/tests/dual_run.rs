//! Dual-run exit rig for the MiniVInst emitter (M5).
//!
//! Every emitted program runs on `lp-xt-emu` against a Rust-computed known
//! answer. When `XT_DEVICE_PORT` is set, each position-independent program
//! *also* runs on the real ESP32-S3 via `xt-runner-client` and the outcomes
//! must agree — the hardware oracle catches any bug the emulator and emitter
//! might share.
//!
//! Hardware tests share ONE board — run single-threaded:
//!   XT_DEVICE_PORT=/dev/cu.usbmodem1101 cargo test -p xt-mini-emit -- --test-threads=1 --nocapture
//! (All device cases live in the single `corpus_dual_run` test; the other
//! tests are emulator-only.)

use lp_xt_emu::emu::{RunOutcome as EmuOutcome, CODE_DBUS_BASE};
use lp_xt_emu::memory::Memory;
use lp_xt_emu::Emulator;

use xt_mini_emit::{
    emit_program, AluImmOp, AluOp, Callee, EmitOut, IcmpCond, MiniFunc, MiniProgram, MiniVInst,
    PReg,
};
use xt_runner_client::{RunOutcome as HwOutcome, Runner};

use MiniVInst::{
    AluRRI, AluRRR, Br, BrIf, Call, FuelCheck, IConst32, Icmp, IcmpImm, Label, Load32, Ret, Select,
    SlotAddr, Store16, Store32, Store8,
};

fn p(n: u8) -> PReg {
    PReg(n)
}

fn func(slots: Vec<u32>, insts: Vec<MiniVInst>) -> MiniFunc {
    MiniFunc { slots, insts }
}

fn prog1(insts: Vec<MiniVInst>) -> MiniProgram {
    MiniProgram {
        funcs: vec![func(vec![], insts)],
    }
}

// ---------------------------------------------------------------------------
// Harness (pattern shared with lp-xt-emu/tests/conformance.rs)
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

/// Emulator vs known answer, then (with a device) emulator vs hardware.
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
// Program corpus
// ---------------------------------------------------------------------------

/// f(a) = 3*(a+5) - 2 (addi + pooled-free constants + mul).
fn prog_arith() -> MiniProgram {
    prog1(vec![
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(3),
            src: p(2),
            imm: 5,
        },
        IConst32 { dst: p(4), val: 3 },
        AluRRR {
            op: AluOp::Mul,
            dst: p(3),
            src1: p(3),
            src2: p(4),
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(3),
            src: p(3),
            imm: -2,
        },
        Ret { val: Some(p(3)) },
    ])
}

/// f(a) = (a + 0x12345678) ^ 0x0F0F0F0F (literal-pool constants; still
/// position-independent — pooled values are constants, not addresses).
fn prog_bigconst() -> MiniProgram {
    prog1(vec![
        IConst32 {
            dst: p(3),
            val: 0x12345678,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(2),
            src2: p(3),
        },
        AluRRI {
            op: AluImmOp::Xori,
            dst: p(4),
            src: p(4),
            imm: 0x0F0F0F0F,
        },
        Ret { val: Some(p(4)) },
    ])
}

/// Shift/div/rem/mulh/and mix (SAR-based shifts, extui path, quou/remu).
fn prog_opsmix() -> MiniProgram {
    prog1(vec![
        IConst32 { dst: p(3), val: 3 },
        AluRRR {
            op: AluOp::Sll,
            dst: p(4),
            src1: p(2),
            src2: p(3),
        },
        AluRRI {
            op: AluImmOp::SrliU,
            dst: p(4),
            src: p(4),
            imm: 1,
        },
        AluRRI {
            op: AluImmOp::SrliU,
            dst: p(5),
            src: p(2),
            imm: 17,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(5),
        },
        IConst32 { dst: p(6), val: 7 },
        AluRRR {
            op: AluOp::DivU,
            dst: p(5),
            src1: p(2),
            src2: p(6),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(5),
        },
        AluRRR {
            op: AluOp::RemU,
            dst: p(5),
            src1: p(2),
            src2: p(6),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(5),
        },
        AluRRI {
            op: AluImmOp::SraiS,
            dst: p(5),
            src: p(2),
            imm: 4,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(5),
        },
        AluRRR {
            op: AluOp::MulH,
            dst: p(5),
            src1: p(2),
            src2: p(2),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(5),
        },
        AluRRI {
            op: AluImmOp::Andi,
            dst: p(5),
            src: p(2),
            imm: 0xFF,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(4),
            src2: p(5),
        },
        Ret { val: Some(p(4)) },
    ])
}

fn opsmix_expect(a: u32) -> u32 {
    let mut r = (a << 3) >> 1;
    r = r.wrapping_add(a >> 17);
    r = r.wrapping_add(a / 7);
    r = r.wrapping_add(a % 7);
    r = r.wrapping_add(((a as i32) >> 4) as u32);
    r = r.wrapping_add((((a as i32 as i64) * (a as i32 as i64)) >> 32) as u32);
    r.wrapping_add(a & 0xFF)
}

/// f(a) = sum(1..=a): counted loop — backward `j`, forward `bnez`, icmp.
fn prog_sumloop() -> MiniProgram {
    prog1(vec![
        IConst32 { dst: p(3), val: 0 }, // acc
        IConst32 { dst: p(4), val: 0 }, // i
        Label(0),
        Icmp {
            dst: p(5),
            lhs: p(4),
            rhs: p(2),
            cond: IcmpCond::GeU,
        },
        BrIf {
            cond: p(5),
            target: 1,
            invert: false,
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(4),
            src: p(4),
            imm: 1,
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(3),
            src1: p(3),
            src2: p(4),
        },
        Br { target: 0 },
        Label(1),
        Ret { val: Some(p(3)) },
    ])
}

/// f(a) = if a >= 10 (signed) { 2 } else { 1 } (forward conditional).
fn prog_branchdir() -> MiniProgram {
    prog1(vec![
        IcmpImm {
            dst: p(3),
            src: p(2),
            imm: 10,
            cond: IcmpCond::GeS,
        },
        BrIf {
            cond: p(3),
            target: 0,
            invert: false,
        },
        IConst32 { dst: p(4), val: 1 },
        Ret { val: Some(p(4)) },
        Label(0),
        IConst32 { dst: p(4), val: 2 },
        Ret { val: Some(p(4)) },
    ])
}

/// f(a) = max_signed(a, 100) via Icmp + Select.
fn prog_select_max() -> MiniProgram {
    prog1(vec![
        IConst32 {
            dst: p(3),
            val: 100,
        },
        Icmp {
            dst: p(4),
            lhs: p(2),
            rhs: p(3),
            cond: IcmpCond::GtS,
        },
        Select {
            dst: p(5),
            cond: p(4),
            if_true: p(2),
            if_false: p(3),
        },
        Ret { val: Some(p(5)) },
    ])
}

const ICMP_CONDS: [IcmpCond; 10] = [
    IcmpCond::Eq,
    IcmpCond::Ne,
    IcmpCond::LtS,
    IcmpCond::LeS,
    IcmpCond::GtS,
    IcmpCond::GeS,
    IcmpCond::LtU,
    IcmpCond::LeU,
    IcmpCond::GtU,
    IcmpCond::GeU,
];

/// f(a) = bitmask of all ten conditions `a COND 10`.
fn prog_icmp_matrix() -> MiniProgram {
    let mut insts = vec![
        IConst32 { dst: p(3), val: 0 },
        IConst32 { dst: p(4), val: 10 },
    ];
    for (i, cond) in ICMP_CONDS.into_iter().enumerate() {
        insts.push(Icmp {
            dst: p(5),
            lhs: p(2),
            rhs: p(4),
            cond,
        });
        insts.push(AluRRI {
            op: AluImmOp::Slli,
            dst: p(5),
            src: p(5),
            imm: i as i32,
        });
        insts.push(AluRRR {
            op: AluOp::Or,
            dst: p(3),
            src1: p(3),
            src2: p(5),
        });
    }
    insts.push(Ret { val: Some(p(3)) });
    prog1(insts)
}

fn icmp_matrix_expect(a: u32) -> u32 {
    let (s, b) = (a as i32, 10i32);
    let bits = [
        s == b,
        s != b,
        s < b,
        s <= b,
        s > b,
        s >= b,
        a < 10,
        a <= 10,
        a > 10,
        a >= 10,
    ];
    bits.iter()
        .enumerate()
        .fold(0u32, |acc, (i, &hit)| acc | ((hit as u32) << i))
}

/// Stack-slot round-trip: word/byte/halfword stores + word loads via SlotAddr.
/// f(a) = a + ((a & 0xFF) | ((a & 0xFFFF) << 16)).
fn prog_slots() -> MiniProgram {
    MiniProgram {
        funcs: vec![func(
            vec![4, 8],
            vec![
                SlotAddr { dst: p(3), slot: 1 },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: 0,
                },
                IConst32 { dst: p(4), val: 0 },
                Store32 {
                    src: p(4),
                    base: p(3),
                    offset: 4,
                },
                Store8 {
                    src: p(2),
                    base: p(3),
                    offset: 4,
                },
                Store16 {
                    src: p(2),
                    base: p(3),
                    offset: 6,
                },
                Load32 {
                    dst: p(5),
                    base: p(3),
                    offset: 0,
                },
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
                Ret { val: Some(p(5)) },
            ],
        )],
    }
}

fn slots_expect(a: u32) -> u32 {
    a.wrapping_add((a & 0xFF) | ((a & 0xFFFF) << 16))
}

/// Two functions in one buffer: main calls mul3 via PC-relative CALL8 (the
/// dual-runnable "builtin" — the runner firmware exposes no host builtins,
/// and CALLX8 needs an absolute address unknowable before load).
/// f(a) = 3*(a+1) + a.
fn prog_call_local() -> MiniProgram {
    let main = func(
        vec![],
        vec![
            AluRRI {
                op: AluImmOp::Addi,
                dst: p(3),
                src: p(2),
                imm: 1,
            },
            Call {
                callee: Callee::Func(1),
                args: vec![p(3)],
                ret: Some(p(4)),
            },
            AluRRR {
                op: AluOp::Add,
                dst: p(4),
                src1: p(4),
                src2: p(2),
            },
            Ret { val: Some(p(4)) },
        ],
    );
    let mul3 = func(
        vec![],
        vec![
            IConst32 { dst: p(3), val: 3 },
            AluRRR {
                op: AluOp::Mul,
                dst: p(2),
                src1: p(2),
                src2: p(3),
            },
            Ret { val: Some(p(2)) },
        ],
    );
    MiniProgram {
        funcs: vec![main, mul3],
    }
}

/// Self-recursion via CALL8: f(d) = d. Depth > 16 forces window
/// overflow/underflow round-trips through the emitted ENTRY frames.
fn prog_recursion() -> MiniProgram {
    prog1(vec![
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
    ])
}

/// Deep recursion with a live stack slot per frame: stores `d` to slot 0,
/// recurses, reloads the slot, and adds `reload - d` (0 iff the slot survived
/// the window spill/reload traffic around it). f(d) = d, but only if the
/// frame layout keeps slots clear of the save areas — this is the hardware
/// check of the `SAVE_AREA_BYTES` reservation.
fn prog_recursion_slots() -> MiniProgram {
    MiniProgram {
        funcs: vec![func(
            vec![4],
            vec![
                BrIf {
                    cond: p(2),
                    target: 0,
                    invert: false,
                },
                IConst32 { dst: p(3), val: 0 },
                Ret { val: Some(p(3)) },
                Label(0),
                SlotAddr { dst: p(3), slot: 0 },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: 0,
                },
                AluRRI {
                    op: AluImmOp::Addi,
                    dst: p(4),
                    src: p(2),
                    imm: -1,
                },
                Call {
                    callee: Callee::Func(0),
                    args: vec![p(4)],
                    ret: Some(p(5)),
                },
                SlotAddr { dst: p(3), slot: 0 },
                Load32 {
                    dst: p(6),
                    base: p(3),
                    offset: 0,
                },
                // p5 += (reload - d) + 1  == p5 + 1 iff the slot survived.
                AluRRR {
                    op: AluOp::Sub,
                    dst: p(6),
                    src1: p(6),
                    src2: p(2),
                },
                AluRRR {
                    op: AluOp::Add,
                    dst: p(5),
                    src1: p(5),
                    src2: p(6),
                },
                AluRRI {
                    op: AluImmOp::Addi,
                    dst: p(5),
                    src: p(5),
                    imm: 1,
                },
                Ret { val: Some(p(5)) },
            ],
        )],
    }
}

/// FuelCheck loop: fuel word (a stack slot, the mini vmctx) starts at `a`;
/// an otherwise-infinite loop increments a counter until the entry-style
/// check observes 0 and branches to the trap label. f(a) = a (iterations
/// completed == initial fuel, proving check-then-decrement ordering).
fn prog_fuel() -> MiniProgram {
    MiniProgram {
        funcs: vec![func(
            vec![4],
            vec![
                SlotAddr { dst: p(3), slot: 0 },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: 0,
                },
                // Counter must be defined before any path to the trap label
                // (the emulator's zeroed registers would mask the read of an
                // uninitialized register; hardware caught exactly that in an
                // earlier revision of this program).
                IConst32 { dst: p(4), val: 0 },
                // Function-entry check (decrement: false, real-IR style).
                FuelCheck {
                    fuel_base: p(3),
                    decrement: false,
                    trap_label: 1,
                },
                Label(0),
                FuelCheck {
                    fuel_base: p(3),
                    decrement: true,
                    trap_label: 1,
                },
                AluRRI {
                    op: AluImmOp::Addi,
                    dst: p(4),
                    src: p(4),
                    imm: 1,
                },
                Br { target: 0 },
                Label(1),
                Ret { val: Some(p(4)) },
            ],
        )],
    }
}

/// Forward conditional across ~2.7 KB of straight-line code — exercises the
/// out-of-range relaxation (inverted beqz over `J`) end to end.
/// f(0) = 900, f(!=0) = 0.
fn prog_longbranch() -> MiniProgram {
    let mut insts = vec![
        IConst32 { dst: p(3), val: 0 },
        BrIf {
            cond: p(2),
            target: 0,
            invert: false,
        },
    ];
    for _ in 0..900 {
        insts.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(3),
            src: p(3),
            imm: 1,
        });
    }
    insts.push(Label(0));
    insts.push(Ret { val: Some(p(3)) });
    prog1(insts)
}

/// CALLX8 through a pooled absolute-address literal — the real builtin-call
/// path. Position-dependent: the host must patch the sym slot with the
/// callee's absolute address before execution. f(a) = 3*a + 1.
fn prog_call_sym() -> MiniProgram {
    use xt_mini_emit::SymbolId;
    let main = func(
        vec![],
        vec![
            Call {
                callee: Callee::Sym(SymbolId(0)),
                args: vec![p(2)],
                ret: Some(p(3)),
            },
            AluRRI {
                op: AluImmOp::Addi,
                dst: p(3),
                src: p(3),
                imm: 1,
            },
            Ret { val: Some(p(3)) },
        ],
    );
    let mul3 = func(
        vec![],
        vec![
            IConst32 { dst: p(3), val: 3 },
            AluRRR {
                op: AluOp::Mul,
                dst: p(2),
                src1: p(2),
                src2: p(3),
            },
            Ret { val: Some(p(2)) },
        ],
    );
    MiniProgram {
        funcs: vec![main, mul3],
    }
}

/// Link `out` for a buffer loaded at 4-aligned absolute address `base`:
/// patch every sym slot with the absolute address of the named local function.
fn link_syms(out: &EmitOut, base: u32) -> Vec<u8> {
    assert_eq!(base % 4, 0, "load address must be 4-aligned");
    let mut code = out.code.clone();
    for &(sym, slot) in &out.sym_slots {
        // In this corpus, SymbolId(n) names funcs[n + 1] (the "builtins").
        let target = base.wrapping_add(out.func_offsets[sym.0 as usize + 1]);
        code[slot as usize..slot as usize + 4].copy_from_slice(&target.to_le_bytes());
    }
    code
}

// ---------------------------------------------------------------------------
// The single dual-run test (device cases must not run concurrently).
// ---------------------------------------------------------------------------

#[test]
fn corpus_dual_run() {
    let mut dev = device();
    let mut seq = 1000u32;
    let args = [0u32, 1, 7, 10, 42, 1000, 0xFFFF_FFFB, 0x8000_0000];

    let arith = emit_program(&prog_arith());
    for a in args {
        let expect = a.wrapping_add(5).wrapping_mul(3).wrapping_sub(2);
        dual_run(&mut dev, &mut seq, "arith", &arith, a, expect);
    }

    let bigconst = emit_program(&prog_bigconst());
    for a in args {
        let expect = a.wrapping_add(0x1234_5678) ^ 0x0F0F_0F0F;
        dual_run(&mut dev, &mut seq, "bigconst", &bigconst, a, expect);
    }

    let opsmix = emit_program(&prog_opsmix());
    for a in args {
        dual_run(&mut dev, &mut seq, "opsmix", &opsmix, a, opsmix_expect(a));
    }

    let sumloop = emit_program(&prog_sumloop());
    for a in [0u32, 1, 10, 50] {
        dual_run(&mut dev, &mut seq, "sumloop", &sumloop, a, a * (a + 1) / 2);
    }

    let branchdir = emit_program(&prog_branchdir());
    for (a, expect) in [(5u32, 1u32), (20, 2), (10, 2), (0xFFFF_FFFB, 1)] {
        dual_run(&mut dev, &mut seq, "branchdir", &branchdir, a, expect);
    }

    let select_max = emit_program(&prog_select_max());
    for a in args {
        let expect = (a as i32).max(100) as u32;
        dual_run(&mut dev, &mut seq, "select-max", &select_max, a, expect);
    }

    let icmp = emit_program(&prog_icmp_matrix());
    for a in [0u32, 9, 10, 11, 0xFFFF_FFFB, 0x8000_0000] {
        dual_run(
            &mut dev,
            &mut seq,
            "icmp-matrix",
            &icmp,
            a,
            icmp_matrix_expect(a),
        );
    }

    let slots = emit_program(&prog_slots());
    for a in args {
        dual_run(&mut dev, &mut seq, "slots", &slots, a, slots_expect(a));
    }

    let call_local = emit_program(&prog_call_local());
    for a in [0u32, 1, 41, 1000] {
        let expect = a.wrapping_add(1).wrapping_mul(3).wrapping_add(a);
        dual_run(&mut dev, &mut seq, "call-local", &call_local, a, expect);
    }

    // Deep recursion: > 16 frames wraps the 64-register file repeatedly,
    // forcing WindowOverflow/Underflow spills through our emitted frames.
    let rec = emit_program(&prog_recursion());
    for d in [0u32, 1, 7, 8, 9, 16, 17, 30, 100] {
        dual_run(&mut dev, &mut seq, "recursion", &rec, d, d);
    }

    // Same depths with a live stack slot in every frame: proves slots stay
    // clear of the window save areas across spill/reload.
    let rec_slots = emit_program(&prog_recursion_slots());
    for d in [0u32, 1, 8, 17, 30, 100] {
        dual_run(&mut dev, &mut seq, "recursion-slots", &rec_slots, d, d);
    }

    let fuel = emit_program(&prog_fuel());
    for f in [0u32, 1, 5, 37] {
        dual_run(&mut dev, &mut seq, "fuel", &fuel, f, f);
    }

    let longbranch = emit_program(&prog_longbranch());
    dual_run(&mut dev, &mut seq, "longbranch-taken", &longbranch, 1, 0);
    dual_run(
        &mut dev,
        &mut seq,
        "longbranch-fallthrough",
        &longbranch,
        0,
        900,
    );

    // CALLX8 to a "builtin" via a pooled ABSOLUTE-address literal. Emulator-only
    // by design: the target address isn't knowable until the buffer is loaded,
    // and the runner loads payloads at heap-chosen addresses that aren't fixed
    // (FINDINGS.md GV3). Device-side calls are already covered by the
    // PC-relative `call-local` case above (CALL8, position-independent). The
    // sym-slot linking mechanism is exercised here and in `callx8_sym_builtin_emu`.
    let sym = emit_program(&prog_call_sym());
    let emu_code = link_syms(&sym, Memory::ibus_alias(CODE_DBUS_BASE));
    for a in [0u32, 1, 14, 1000] {
        let expect = a.wrapping_mul(3).wrapping_add(1);
        match emu_run(&emu_code, sym.entry_offset, a) {
            EmuOutcome::Ok(v) => assert_eq!(v, expect, "[call-sym] emu f({a})"),
            other => panic!("[call-sym] emu a={a} unexpected: {other:?}"),
        }
    }

    eprintln!(
        "corpus_dual_run: all cases passed (device={})",
        dev.is_some()
    );
}

// ---------------------------------------------------------------------------
// Emulator-only: the CALLX8 sym-literal path as a standalone test (also part
// of the corpus above), so it runs even when the corpus is filtered out.
// ---------------------------------------------------------------------------

#[test]
fn callx8_sym_builtin_emu() {
    let out = emit_program(&prog_call_sym());
    assert_eq!(out.sym_slots.len(), 1);
    let code = link_syms(&out, Memory::ibus_alias(CODE_DBUS_BASE));
    for a in [0u32, 1, 14, 1000] {
        match emu_run(&code, out.entry_offset, a) {
            EmuOutcome::Ok(v) => assert_eq!(v, 3 * a + 1, "callx8 f({a})"),
            other => panic!("callx8 a={a} unexpected: {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Emulator-only: window-trap visibility — deep recursion through emitted
// frames must actually spill and reload (not merely produce the right value).
// ---------------------------------------------------------------------------

#[test]
fn recursion_forces_window_traps() {
    use lp_xt_emu::TextTracer;
    let out = emit_program(&prog_recursion());
    let mut emu = Emulator::new();
    let mut tr = TextTracer::new();
    let res = emu.run_traced(&out.code, out.entry_offset, 40, &mut tr);
    assert_eq!(res, EmuOutcome::Ok(40));
    let text = tr.dump();
    assert!(text.contains("spill"), "depth-40 recursion must spill");
    assert!(text.contains("reload"), "depth-40 recursion must reload");
}
