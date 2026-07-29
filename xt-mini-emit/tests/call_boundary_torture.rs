//! P3 — Call-boundary register contract torture + divide-by-zero trap parity.
//!
//! The monorepo's register allocator will *depend* on the CALL8 contract
//! pinned in `xt-mini-emit/src/gpr.rs`: `a2..=a7` survive a call, `a8..=a15`
//! are clobbered (`a_j` survives iff `j < 8`). This suite tortures that
//! contract on the emulator (every board profile) and on every attached
//! board (`XT_PORT_ESP32S3` / `XT_PORT_ESP32`):
//!
//! 1. Plant a distinct known value in **every** register `a2..=a15`, make a
//!    CALL8 into callees of varying shape — leaf clobberer, non-leaf
//!    clobberer (a callee that itself calls), and self-recursion at depths
//!    crossing the window-overflow threshold — then verify the preserved bank
//!    after return. One fold witness (`sum of prime * reg`) checks all six
//!    preserved registers in a single returned u32; per-register probes name
//!    any offender and measure the clobbered bank's survival *predicate*
//!    (never its raw value: post-call `a8` holds the CALLINC-mangled return
//!    address, which is load-address-dependent — P1's lesson).
//! 2. The caller-saved-spill pattern the allocator relies on: a value live in
//!    a clobbered register across a call is spilled to a stack slot by the
//!    caller and reloaded after — verified against active clobberers and
//!    deep recursion.
//! 3. Divide-by-zero trap parity: emitter-built `quos/quou/rems/remu` with a
//!    zero divisor must crash with the same class (kind + EXCCAUSE) on the
//!    emulator and the device, plus the INT_MIN / -1 overflow value edge.
//!
//! All programs are built through `lp_xt_inst::encode` or the emitter — no
//! hand-encoded bytes. Position-independent (PC-relative CALL8 only), so
//! every case dual-runs on hardware.
//!
//! Hardware tests share the boards — run single-threaded:
//!   XT_PORT_ESP32S3=... XT_PORT_ESP32=... cargo test -p xt-mini-emit -- --test-threads=1 --nocapture

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::error::EXC_INTEGER_DIVIDE_BY_ZERO;
use lp_xt_emu::{Emulator, TextTracer, TrapKind};
use lp_xt_inst::{encode, AluRrr, BrZ, CallOp, Inst, LoadOp, NullaryNarrowOp, NullaryOp, Reg, StoreOp};

use xt_mini_emit::{emit_program, gpr, AluOp, MiniFunc, MiniProgram, MiniVInst, PReg};
use xt_testkit::{Harness, Outcome};

// ---------------------------------------------------------------------------
// Harness (shared: xt-testkit N-runs each case on every profile + board;
// `Harness::run_all` gives per-world values where only a predicate of a
// position-dependent value is comparable)
// ---------------------------------------------------------------------------

fn measure(h: &mut Harness, name: &str, code: &[u8], entry_offset: u32, arg: u32, expect: u32) {
    h.nrun(name, code, entry_offset, arg, expect);
}

// ---------------------------------------------------------------------------
// Raw program builder (encode-built; registers outside the MiniVInst program
// range a2..=a7 require assembling directly, as P1's raw probes did)
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Raw {
    code: Vec<u8>,
}

impl Raw {
    fn inst(&mut self, i: Inst) {
        self.code.extend(encode(&i));
    }

    fn here(&self) -> u32 {
        self.code.len() as u32
    }

    /// Pad to 4-byte alignment with executable nops (CALLn targets are
    /// computed `(PC & !3) + (off << 2) + 4`, so entries must be 4-aligned).
    fn align4(&mut self) {
        match self.code.len() % 4 {
            0 => {}
            2 => self.inst(Inst::NullaryN(NullaryNarrowOp::NopN)),
            3 => self.inst(Inst::Nullary(NullaryOp::Nop)),
            1 => {
                self.inst(Inst::Nullary(NullaryOp::Nop));
                self.inst(Inst::NullaryN(NullaryNarrowOp::NopN));
            }
            _ => unreachable!(),
        }
    }

    /// PC-relative `call8` to a 4-aligned offset already in this buffer.
    fn call8(&mut self, target: u32) {
        let pc = self.code.len() as i64;
        let t = target as i64;
        let off = (t - (pc & !3) - 4) >> 2;
        assert_eq!((pc & !3) + (off << 2) + 4, t, "call8 target must be exact");
        self.inst(Inst::Call(CallOp::Call8, off as i32));
    }
}

fn r(n: u8) -> Reg {
    Reg::new(n)
}

// ---------------------------------------------------------------------------
// Torture programs
// ---------------------------------------------------------------------------

/// Callee shape on the far side of the CALL8 under test.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shape {
    /// Leaf that actively writes junk into its whole window (`a2..=a15`).
    Leaf,
    /// Clobberer that itself CALL8s the leaf clobberer.
    NonLeaf,
    /// Self-recursion to the given depth, each frame junking its window.
    /// Depths past the overflow onset (6 frames) cover the hardware
    /// spill/reload path for the planted caller registers.
    Rec(u32),
}

impl Shape {
    fn name(self) -> String {
        match self {
            Shape::Leaf => "leaf".into(),
            Shape::NonLeaf => "nonleaf".into(),
            Shape::Rec(d) => format!("rec{d}"),
        }
    }
}

/// Planted value for `a{reg}` in the witness main (distinct per register,
/// disjoint from every callee's junk ranges).
fn k_val(reg: u8) -> i32 {
    401 + 13 * reg as i32
}

/// Distinct primes folding `a2..=a7` into one returned u32.
const PRIMES: [u32; 6] = [3, 5, 7, 11, 13, 17];

/// Leaf clobberer: `entry; movi a2..a15, junk; retw`. Writing its full window
/// genuinely overwrites the caller's `a10..=a15` (the rotation overlap) plus
/// the 8 physical registers above them.
fn emit_leaf_clobber(raw: &mut Raw) -> u32 {
    raw.align4();
    let entry = raw.here();
    raw.inst(Inst::Entry(r(1), 32));
    for reg in 2u8..=15 {
        raw.inst(Inst::Movi(r(reg), 1100 + 7 * reg as i32));
    }
    raw.inst(Inst::Nullary(NullaryOp::Retw));
    entry
}

/// Non-leaf clobberer: junks its window, then CALL8s the leaf clobberer.
fn emit_nonleaf_clobber(raw: &mut Raw, leaf: u32) -> u32 {
    raw.align4();
    let entry = raw.here();
    raw.inst(Inst::Entry(r(1), 32));
    for reg in 2u8..=15 {
        raw.inst(Inst::Movi(r(reg), 1300 + 7 * reg as i32));
    }
    raw.call8(leaf);
    raw.inst(Inst::Nullary(NullaryOp::Retw));
    entry
}

/// Self-recursion `f(d)`: junks every window register it may (all but RA/SP,
/// the depth arg `a2`, and the `a10` staging slot), then recurses. Depth > ~5
/// wraps the 64-entry physical file, so descendants physically overwrite the
/// planted caller window, and only the overflow/underflow save-area round-trip
/// can restore it.
fn emit_rec(raw: &mut Raw) -> u32 {
    raw.align4();
    let entry = raw.here();
    raw.inst(Inst::Entry(r(1), 32));
    for reg in [3u8, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15] {
        raw.inst(Inst::Movi(r(reg), 1500 + 7 * reg as i32));
    }
    // if depth == 0, skip the recurse (addi 3B + call8 3B): diff = 9 - 4.
    raw.inst(Inst::BranchZ(BrZ::Beqz, r(2), 5));
    raw.inst(Inst::Addi(r(10), r(2), -1));
    raw.call8(entry);
    raw.inst(Inst::Nullary(NullaryOp::Retw));
    entry
}

fn emit_shape(raw: &mut Raw, shape: Shape) -> u32 {
    match shape {
        Shape::Leaf => emit_leaf_clobber(raw),
        Shape::NonLeaf => {
            let leaf = emit_leaf_clobber(raw);
            emit_nonleaf_clobber(raw, leaf)
        }
        Shape::Rec(_) => emit_rec(raw),
    }
}

/// The value actually planted in `a{reg}` at the CALL8 site (for `Rec`, the
/// staging register `a10` carries the depth — it is clobbered-class anyway).
fn planted(shape: Shape, reg: u8) -> u32 {
    match shape {
        Shape::Rec(d) if reg == 10 => d,
        _ => k_val(reg) as u32,
    }
}

enum Finish {
    /// Return `sum(PRIMES[r-2] * a_r for r in 2..=7)` — one u32 witnessing
    /// the whole preserved bank.
    Fold,
    /// Return `a{reg}` — the per-register bisection probe.
    Probe(u8),
}

/// Witness main: plant `a2..=a15`, CALL8 into `shape`, then run `finish`.
/// Returns `(code, entry_offset, expected/planted value)`.
fn build_witness(shape: Shape, finish: Finish) -> (Vec<u8>, u32, u32) {
    let mut raw = Raw::default();
    let target = emit_shape(&mut raw, shape);
    raw.align4();
    let entry = raw.here();
    raw.inst(Inst::Entry(r(1), 32));
    for reg in 2u8..=15 {
        raw.inst(Inst::Movi(r(reg), k_val(reg)));
    }
    if let Shape::Rec(d) = shape {
        raw.inst(Inst::Movi(r(10), d as i32)); // stage the depth argument
    }
    raw.call8(target);
    let expect = match finish {
        Finish::Fold => {
            // a8/a9 are dead after the call (emitter-scratch class) — fold
            // there: a8 = acc, a9 = prime * reg.
            raw.inst(Inst::Movi(r(8), 0));
            let mut sum = 0u32;
            for reg in 2u8..=7 {
                let prime = PRIMES[(reg - 2) as usize];
                raw.inst(Inst::Movi(r(9), prime as i32));
                raw.inst(Inst::Rrr(AluRrr::Mull, r(9), r(9), r(reg)));
                raw.inst(Inst::Rrr(AluRrr::Add, r(8), r(8), r(9)));
                sum = sum.wrapping_add(prime.wrapping_mul(k_val(reg) as u32));
            }
            raw.inst(Inst::Rrr(AluRrr::Or, r(2), r(8), r(8)));
            sum
        }
        Finish::Probe(reg) => {
            raw.inst(Inst::Rrr(AluRrr::Or, r(2), r(reg), r(reg)));
            planted(shape, reg)
        }
    };
    raw.inst(Inst::Nullary(NullaryOp::Retw));
    (raw.code, entry, expect)
}

/// The caller-saved-spill pattern the allocator depends on: a value live in
/// clobbered `a10` across a call is spilled to a stack slot by the caller and
/// reloaded after. `f(arg) = arg` iff the spill/reload round-trip works.
fn build_caller_spill(shape: Shape) -> (Vec<u8>, u32) {
    let mut raw = Raw::default();
    let target = emit_shape(&mut raw, shape);
    raw.align4();
    let entry = raw.here();
    raw.inst(Inst::Entry(r(1), 48)); // 32 B save areas at top + 16 B slots
    raw.inst(Inst::Rrr(AluRrr::Or, r(10), r(2), r(2))); // live value in a10
    raw.inst(Inst::Store(StoreOp::S32i, r(10), r(1), 0)); // caller spills it
    if let Shape::Rec(d) = shape {
        raw.inst(Inst::Movi(r(10), d as i32)); // a10 re-used as the call arg
    }
    raw.call8(target);
    raw.inst(Inst::Load(LoadOp::L32i, r(8), r(1), 0)); // caller reloads
    raw.inst(Inst::Rrr(AluRrr::Or, r(2), r(8), r(8)));
    raw.inst(Inst::Nullary(NullaryOp::Retw));
    (raw.code, entry)
}

// ---------------------------------------------------------------------------
// The single dual-run test (device cases must not run concurrently)
// ---------------------------------------------------------------------------

const SHAPES: [Shape; 6] = [
    Shape::Leaf,
    Shape::NonLeaf,
    Shape::Rec(1),
    Shape::Rec(5),
    Shape::Rec(8),
    Shape::Rec(40),
];

#[test]
fn call_boundary_contract_torture() {
    let mut h = Harness::from_env(8000);

    for shape in SHAPES {
        let name = shape.name();

        // 1) Fold witness: one u32 checks the whole preserved bank at once.
        let (code, entry, expect) = build_witness(shape, Finish::Fold);
        measure(&mut h, &format!("fold-{name}"), &code, entry, 0, expect);

        // 2) Per-register probes: name any offender, and measure the
        //    clobbered bank's survival predicate.
        let mut survived: Vec<u8> = Vec::new();
        for reg in 2u8..=15 {
            let (code, entry, planted) = build_witness(shape, Finish::Probe(reg));
            let pname = format!("probe-{name}-a{reg}");
            if gpr::is_callee_saved_pool(reg) {
                // Contract-preserved: the planted value is deterministic and
                // MUST come back exactly in every world.
                measure(&mut h, &pname, &code, entry, 0, planted);
                survived.push(reg);
            } else {
                // Contract-clobbered: raw post-call values may be position-
                // dependent (mangled RA in a8, callee SP in a9) and so differ
                // by world; every world must agree on the survival
                // *predicate* only.
                let results = h.run_all(&pname, &code, entry, 0);
                let alive = results[0].1 == planted;
                for (world, v) in &results[1..] {
                    assert_eq!(
                        *v == planted,
                        alive,
                        "[{pname}] survival disagree: {}={:#010x} vs {world}={v:#010x} \
                         (planted={planted:#x})",
                        results[0].0, results[0].1
                    );
                }
                if alive {
                    survived.push(reg);
                }
            }
        }

        // 3) Model assertions against the contract module itself:
        //    - every contract-preserved register survived (asserted above);
        //    - every register that did NOT survive is one the contract
        //      already declares dead across a call (scratch or caller-saved)
        //      — i.e. the emitter/allocator never relies on it.
        for reg in 2u8..=15 {
            if !survived.contains(&reg) {
                assert!(
                    !gpr::is_callee_saved_pool(reg),
                    "[{name}] contract-preserved a{reg} was clobbered — CONTRACT VIOLATION"
                );
                assert!(
                    reg == gpr::SCRATCH
                        || reg == gpr::SCRATCH2
                        || gpr::is_caller_saved_pool(reg),
                    "[{name}] a{reg} unaccounted for by the contract"
                );
            }
        }
        // An *active* leaf clobberer overwrites everything it can reach, so
        // there the survived set must be EXACTLY the contract's preserved
        // bank (incidental survival is possible only for callees that happen
        // to restore a value, e.g. recursion echoing the depth through a10).
        if shape == Shape::Leaf {
            let contract: Vec<u8> = (2u8..=15).filter(|&x| gpr::is_callee_saved_pool(x)).collect();
            assert_eq!(
                survived, contract,
                "leaf-clobber survived set must equal the contract's preserved bank"
            );
        }
        eprintln!("MEASURE contract shape={name} survived={survived:?} (contract preserved = a2..a7)");
    }

    // 4) Tracer evidence that the deep shapes actually exercised the window
    //    spill/reload path (a preserved-bank check that never triggered the
    //    handlers would prove nothing about it).
    for (shape, want_spills) in [(Shape::Rec(1), false), (Shape::Rec(8), true), (Shape::Rec(40), true)] {
        let (code, entry, expect) = build_witness(shape, Finish::Fold);
        let mut emu = Emulator::new();
        let mut tr = TextTracer::new();
        let res = emu.run_traced(&code, entry, 0, &mut tr);
        assert_eq!(res, EmuOutcome::Ok(expect));
        let spills = tr.lines.iter().filter(|l| l.contains("spill")).count();
        let reloads = tr.lines.iter().filter(|l| l.contains("reload")).count();
        assert_eq!(spills, reloads, "{shape:?}: every spilled frame must reload");
        assert_eq!(
            spills > 0,
            want_spills,
            "{shape:?}: expected spills={want_spills}, counted {spills}"
        );
        eprintln!("MEASURE window shape={} spills={spills} reloads={reloads}", shape.name());
    }

    // 5) Caller-saved-spill pattern (value survives in a stack slot while the
    //    register dies), against the active clobberer and deep recursion.
    for shape in [Shape::Leaf, Shape::Rec(40)] {
        let (code, entry) = build_caller_spill(shape);
        for arg in [0u32, 42, 0xDEAD_BEEF, u32::MAX] {
            let name = format!("caller-spill-{}", shape.name());
            measure(&mut h, &name, &code, entry, arg, arg);
        }
    }
    eprintln!("MEASURE caller-saved-spill: slot round-trip across leaf + rec40 OK");

    // 6) Divide-by-zero trap parity (crash cases last: each hardware crash
    //    resets the device; the client recovers across the reset).
    div_by_zero_parity(&mut h);

    eprintln!(
        "call_boundary_contract_torture: all cases passed (boards={})",
        h.boards.len()
    );
}

// ---------------------------------------------------------------------------
// Divide-by-zero trap parity (P3 scope item 2 / plan Q3)
// ---------------------------------------------------------------------------

fn p(n: u8) -> PReg {
    PReg(n)
}

/// `f(a) = a <op> divisor` through the emitter (registers in the program bank).
fn div_prog(op: AluOp, divisor: i32) -> MiniProgram {
    MiniProgram {
        funcs: vec![MiniFunc {
            slots: vec![],
            insts: vec![
                MiniVInst::IConst32 {
                    dst: p(3),
                    val: divisor,
                },
                MiniVInst::AluRRR {
                    op,
                    dst: p(2),
                    src1: p(2),
                    src2: p(3),
                },
                MiniVInst::Ret { val: Some(p(2)) },
            ],
        }],
    }
}

fn div_by_zero_parity(h: &mut Harness) {
    let ops = [
        (AluOp::DivS, "quos"),
        (AluOp::DivU, "quou"),
        (AluOp::RemS, "rems"),
        (AluOp::RemU, "remu"),
    ];

    // Zero divisor: EVERY world must crash with the same class — an
    // Exception with EXCCAUSE = IntegerDivideByZero (6). This is also the
    // first hardware probe of whether LX6 silicon has quos/quou/rems/remu at
    // all: a missing divider would surface as IllegalInstruction (cause 0)
    // here, not as a wrong value.
    for (op, opname) in ops {
        let out = emit_program(&div_prog(op, 0));
        let name = format!("div0-{opname}");
        for w in h.run_worlds(&name, &out.code, out.entry_offset, 42) {
            match w.outcome {
                Outcome::Crash { kind, cause } => {
                    assert_eq!(
                        kind,
                        TrapKind::Exception,
                        "[{name}] {} crash-class (cause={cause})",
                        w.world
                    );
                    assert_eq!(
                        cause, EXC_INTEGER_DIVIDE_BY_ZERO,
                        "[{name}] {} EXCCAUSE (0 here would mean IllegalInstruction — \
                         no hardware divider on this chip)",
                        w.world
                    );
                    eprintln!("[{name}] {} crash agree: Exception cause={cause}", w.world);
                }
                Outcome::Ok(v) => panic!(
                    "[{name}] {} returned Ok({v}) where a divide-by-zero trap is \
                     required — trap-parity FINDING",
                    w.world
                ),
            }
        }
    }

    // The INT_MIN / -1 overflow edge does NOT divide by zero: it must return
    // a value, and every world must agree on it (the model wraps:
    // quotient INT_MIN, remainder 0).
    for (op, opname, arg, expect) in [
        (AluOp::DivS, "quos", 0x8000_0000u32, 0x8000_0000u32),
        (AluOp::RemS, "rems", 0x8000_0000, 0),
    ] {
        let out = emit_program(&div_prog(op, -1));
        let name = format!("divmin-{opname}");
        measure(h, &name, &out.code, out.entry_offset, arg, expect);
    }
    eprintln!("MEASURE div-by-zero: quos/quou/rems/remu all trap Exception cause=6; INT_MIN/-1 wraps");
}
