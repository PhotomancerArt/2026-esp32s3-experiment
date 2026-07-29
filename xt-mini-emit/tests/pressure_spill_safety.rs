//! P5 — Register pressure characterization + spill-slot vs window-save-area
//! collision torture.
//!
//! Two backport questions, answered with evidence:
//!
//! 1. **Pressure**: how many simultaneously-live u32s fit before spill slots
//!    are needed? Contract answer (`gpr.rs`): the pool is **12** registers
//!    (vs rv32's 13); across a call the free-to-keep set is the preserved
//!    bank of **6** (rv32 keeps 10 callee-saved, paid for with prologue
//!    stores; Xtensa's 6 survive by rotation at zero instruction cost — the
//!    caller-saved 6 need the caller-spill pattern P3 verifies). Programs
//!    below hold vec4-shaped live sets (4/8/12/16/20 across arithmetic,
//!    4/6/8/12/16 across calls), hand-allocated exactly as the monorepo
//!    allocator would (registers in `ALLOC_POOL` order, then slots), and a
//!    prime-weighted / shift-weighted checksum witnesses every value.
//!
//! 2. **Collision torture** (the important half): the window overflow
//!    handlers write ancestor registers into per-frame save areas *unbidden*.
//!    If stack slots ever overlapped them, ancestor frames would corrupt
//!    silently. The M5 `recursion-slots` case is generalized into a matrix
//!    over slot count x depth x frame padding x call increment; every slot of
//!    every frame and every ancestor register is verified after unwinding,
//!    and a collision tracer asserts (a) spills/reloads actually fired and
//!    (b) no program store ever touched a byte the handlers spilled to.
//!
//! Hardware tests share the boards — run single-threaded:
//!   XT_PORT_ESP32S3=... XT_PORT_ESP32=... cargo test -p xt-mini-emit -- --test-threads=1 --nocapture

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::trace::TraceEvent;
use lp_xt_emu::{Emulator, Tracer};
use lp_xt_inst::{encode, AluRrr, Inst, LoadOp, NullaryOp, Reg, StoreOp};

use xt_mini_emit::{
    emit_program_with, gpr, AluImmOp, AluOp, CallInc, Callee, MiniFunc, MiniProgram, MiniVInst,
    PReg,
};
use xt_testkit::Harness;

use MiniVInst::{AluRRI, AluRRR, BrIf, Call, IConst32, Label, Load32, Ret, SlotAddr, Store32};

// ---------------------------------------------------------------------------
// Harness (shared: xt-testkit N-runs each case on every profile + board)
// ---------------------------------------------------------------------------

fn measure(h: &mut Harness, name: &str, code: &[u8], entry_offset: u32, arg: u32, expect: u32) {
    h.nrun(name, code, entry_offset, arg, expect);
}

fn p(n: u8) -> PReg {
    PReg(n)
}

fn r(n: u8) -> Reg {
    Reg::new(n)
}

// ---------------------------------------------------------------------------
// 1a. Pressure across arithmetic (raw-built: uses the full 12-register pool,
//     which spans both banks — outside the MiniVInst program range)
// ---------------------------------------------------------------------------

/// Distinct primes weighting each live value in the fold.
const PRIMES: [u32; 20] = [
    3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73,
];

fn live_k(i: usize) -> i32 {
    700 + 31 * i as i32
}

/// `n` simultaneously live u32s across arithmetic: the first
/// `min(n, 12)` live in registers (in `ALLOC_POOL` order — the allocator's
/// own hand), the rest in stack slots; scratch churn in between; then a
/// prime-weighted fold of every value. Returns `(code, entry, expected)`.
fn build_live_arith(n: usize) -> (Vec<u8>, u32, u32) {
    let pool = gpr::ALLOC_POOL;
    let in_regs = n.min(pool.len());
    let in_slots = n - in_regs;
    let slots_bytes = 4 * in_slots as u32;
    let frame = (32 + slots_bytes).div_ceil(16) * 16;

    let mut code = Vec::new();
    let mut inst = |i: Inst| code.extend(encode(&i));
    inst(Inst::Entry(r(1), frame));
    // Plant the register-resident values.
    for (i, &preg) in pool.iter().take(in_regs).enumerate() {
        inst(Inst::Movi(r(preg), live_k(i)));
    }
    // Plant the slot-resident values through scratch (slots at SP+0 upward).
    for j in 0..in_slots {
        inst(Inst::Movi(r(gpr::SCRATCH), live_k(in_regs + j)));
        inst(Inst::Store(StoreOp::S32i, r(gpr::SCRATCH), r(1), 4 * j as u32));
    }
    // Scratch churn: prove the fold reads live values, not stale scratch.
    inst(Inst::Movi(r(gpr::SCRATCH), 1234));
    inst(Inst::Movi(r(gpr::SCRATCH2), 777));
    inst(Inst::Rrr(AluRrr::Add, r(gpr::SCRATCH), r(gpr::SCRATCH), r(gpr::SCRATCH2)));
    inst(Inst::Rrr(AluRrr::Xor, r(gpr::SCRATCH2), r(gpr::SCRATCH), r(gpr::SCRATCH2)));
    // Fold: acc in SCRATCH, prime/operand staging in SCRATCH2.
    inst(Inst::Movi(r(gpr::SCRATCH), 0));
    let mut expect = 0u32;
    for i in 0..in_regs {
        inst(Inst::Movi(r(gpr::SCRATCH2), PRIMES[i] as i32));
        inst(Inst::Rrr(AluRrr::Mull, r(gpr::SCRATCH2), r(gpr::SCRATCH2), r(pool[i])));
        inst(Inst::Rrr(AluRrr::Add, r(gpr::SCRATCH), r(gpr::SCRATCH), r(gpr::SCRATCH2)));
        expect = expect.wrapping_add(PRIMES[i].wrapping_mul(live_k(i) as u32));
    }
    for j in 0..in_slots {
        // Registers are dead once folded — a2 is free as a third temp.
        inst(Inst::Load(LoadOp::L32i, r(gpr::SCRATCH2), r(1), 4 * j as u32));
        inst(Inst::Movi(r(2), PRIMES[in_regs + j] as i32));
        inst(Inst::Rrr(AluRrr::Mull, r(gpr::SCRATCH2), r(gpr::SCRATCH2), r(2)));
        inst(Inst::Rrr(AluRrr::Add, r(gpr::SCRATCH), r(gpr::SCRATCH), r(gpr::SCRATCH2)));
        expect = expect.wrapping_add(PRIMES[in_regs + j].wrapping_mul(live_k(in_regs + j) as u32));
    }
    inst(Inst::Rrr(AluRrr::Or, r(2), r(gpr::SCRATCH), r(gpr::SCRATCH)));
    inst(Inst::Nullary(NullaryOp::Retw));
    (code, 0, expect)
}

// ---------------------------------------------------------------------------
// 1b. Pressure across a call (through the emitter: values live across a call
//     sit in the preserved bank a2..=a7, overflow goes to slots)
// ---------------------------------------------------------------------------

/// Distinct per-value offsets, addi-range-safe.
fn call_c(i: usize) -> i32 {
    7 * i as i32
}

/// `n` values `v_i = arg + 7i` live across a call into a callee that clobbers
/// its whole program bank: `min(n, 6)` in the preserved registers `a2..=a7`,
/// the rest in stack slots. After the call, fold `sum(v_i << i)`.
/// `f(arg) = sum((arg + 7i) << i)`.
fn prog_live_call(n: usize) -> MiniProgram {
    assert!((2..=16).contains(&n));
    let in_regs = n.min(6); // a2 (the arg itself) .. a7
    let in_slots = n - in_regs;

    let mut insts: Vec<MiniVInst> = Vec::new();
    // Slot values first (a3/a4 as temps, before the register bank is live).
    for s in 0..in_slots {
        insts.push(SlotAddr {
            dst: p(3),
            slot: s as u32,
        });
        insts.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(4),
            src: p(2),
            imm: call_c(in_regs + s),
        });
        insts.push(Store32 {
            src: p(4),
            base: p(3),
            offset: 0,
        });
    }
    // Register values: a2 = v_0 (= arg), a{2+i} = v_i.
    for i in 1..in_regs {
        insts.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(2 + i as u8),
            src: p(2),
            imm: call_c(i),
        });
    }
    insts.push(Call {
        callee: Callee::Func(1),
        args: vec![],
        ret: None,
    });
    // Fold the register bank: acc = a2 += v_i << i (v_i dead after its fold).
    for i in 1..in_regs {
        insts.push(AluRRI {
            op: AluImmOp::Slli,
            dst: p(2 + i as u8),
            src: p(2 + i as u8),
            imm: i as i32,
        });
        insts.push(AluRRR {
            op: AluOp::Add,
            dst: p(2),
            src1: p(2),
            src2: p(2 + i as u8),
        });
    }
    // Fold the slots (a3/a4 free again).
    for s in 0..in_slots {
        insts.push(SlotAddr {
            dst: p(3),
            slot: s as u32,
        });
        insts.push(Load32 {
            dst: p(4),
            base: p(3),
            offset: 0,
        });
        insts.push(AluRRI {
            op: AluImmOp::Slli,
            dst: p(4),
            src: p(4),
            imm: (in_regs + s) as i32,
        });
        insts.push(AluRRR {
            op: AluOp::Add,
            dst: p(2),
            src1: p(2),
            src2: p(4),
        });
    }
    insts.push(Ret { val: Some(p(2)) });

    let clobber = MiniFunc {
        slots: vec![],
        insts: (2u8..=7)
            .map(|i| IConst32 {
                dst: p(i),
                val: 0xC0 + i as i32,
            })
            .chain([Ret { val: None }])
            .collect(),
    };
    MiniProgram {
        funcs: vec![
            MiniFunc {
                slots: vec![4; in_slots],
                insts,
            },
            clobber,
        ],
    }
}

fn live_call_expect(arg: u32, n: usize) -> u32 {
    (0..n).fold(0u32, |acc, i| {
        acc.wrapping_add(arg.wrapping_add(call_c(i) as u32) << i)
    })
}

#[test]
fn pressure_characterization() {
    let mut h = Harness::from_env(9000);

    // Across arithmetic: the achievable pool is ALLOC_POOL's 12 (vs rv32 13).
    for n in [4usize, 8, 12, 16, 20] {
        let (code, entry, expect) = build_live_arith(n);
        let regs = n.min(gpr::ALLOC_POOL.len());
        let slots = n - regs;
        measure(&mut h, &format!("live-arith-{n}"), &code, entry, 0, expect);
        eprintln!(
            "MEASURE pressure-arith live={n} regs={regs} spill_slots={slots} \
             (pool={} vs rv32 13)",
            gpr::ALLOC_POOL.len()
        );
    }

    // Across a call: 6 preserved registers carry values for free; the rest
    // need slots (or the caller-saved spill pattern, P3).
    for n in [4usize, 6, 8, 12, 16] {
        let out = emit_program_with(&prog_live_call(n), CallInc::Call8);
        let regs = n.min(6);
        let slots = n - regs;
        for arg in [0u32, 5, 0xFFFF_FF00] {
            let name = format!("live-call-{n}");
            measure(
                &mut h, &name,
                &out.code,
                out.entry_offset,
                arg,
                live_call_expect(arg, n),
            );
        }
        eprintln!(
            "MEASURE pressure-call live={n} preserved_regs={regs} spill_slots={slots} \
             (preserved bank=6, caller-saved bank=6; rv32 callee-saved pool=10)"
        );
    }

    eprintln!(
        "pressure_characterization: all cases passed (boards={})",
        h.boards.len()
    );
}

// ---------------------------------------------------------------------------
// 2. Spill-slot vs window-save-area collision torture
// ---------------------------------------------------------------------------

/// Generalized `recursion-slots`: `f(d)` writes `d + slot_c(s)` into every
/// live slot, recurses, then verifies EVERY slot and its own `d` (an ancestor
/// register live across the call) after the descendant chain unwinds.
/// `f(d) = d` iff no slot of any frame and no ancestor register was corrupted
/// by the window traffic. An optional unused pad slot below the live slots
/// varies the frame size / slot placement independently of the live count.
fn prog_rec_slots(s_live: usize, pad_bytes: u32) -> MiniProgram {
    assert!(s_live >= 1);
    let mut slots: Vec<u32> = Vec::new();
    let first_live = if pad_bytes > 0 {
        slots.push(pad_bytes);
        1u32
    } else {
        0
    };
    slots.extend(std::iter::repeat_n(4u32, s_live));

    let slot_c = |s: usize| (2 * s as i32) - 63; // distinct, addi-range
    let mut insts: Vec<MiniVInst> = Vec::new();
    // Store phase (also runs in the base-case frame — every frame writes).
    for s in 0..s_live {
        insts.push(SlotAddr {
            dst: p(3),
            slot: first_live + s as u32,
        });
        insts.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(4),
            src: p(2),
            imm: slot_c(s),
        });
        insts.push(Store32 {
            src: p(4),
            base: p(3),
            offset: 0,
        });
    }
    insts.push(BrIf {
        cond: p(2),
        target: 0,
        invert: false,
    });
    insts.push(IConst32 { dst: p(5), val: 0 });
    insts.push(Ret { val: Some(p(5)) });
    insts.push(Label(0));
    insts.push(AluRRI {
        op: AluImmOp::Addi,
        dst: p(4),
        src: p(2),
        imm: -1,
    });
    insts.push(Call {
        callee: Callee::Func(0),
        args: vec![p(4)],
        ret: Some(p(5)),
    });
    // Verify phase: acc (a5) += reload - expected, per slot; then +1.
    // Uses a2 (= d) AFTER the deep call — an ancestor-register check too.
    for s in 0..s_live {
        insts.push(SlotAddr {
            dst: p(3),
            slot: first_live + s as u32,
        });
        insts.push(Load32 {
            dst: p(6),
            base: p(3),
            offset: 0,
        });
        insts.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(7),
            src: p(2),
            imm: slot_c(s),
        });
        insts.push(AluRRR {
            op: AluOp::Sub,
            dst: p(6),
            src1: p(6),
            src2: p(7),
        });
        insts.push(AluRRR {
            op: AluOp::Add,
            dst: p(5),
            src1: p(5),
            src2: p(6),
        });
    }
    insts.push(AluRRI {
        op: AluImmOp::Addi,
        dst: p(5),
        src: p(5),
        imm: 1,
    });
    insts.push(Ret { val: Some(p(5)) });

    MiniProgram {
        funcs: vec![MiniFunc { slots, insts }],
    }
}

/// Tracer that records window spill/reload activity AND every guest store, so
/// the test can assert (a) the handlers actually fired and (b) the byte
/// ranges the handlers spilled ancestor registers into are disjoint from
/// every byte the program itself stored (slot writes) — a direct address-
/// level collision check on top of the value checks.
#[derive(Default)]
struct CollisionTracer {
    /// Byte ranges written by window spills: `[sp - 4*nregs, sp)`.
    spill_ranges: Vec<(u32, u32)>,
    spills: usize,
    reloads: usize,
    /// Byte ranges written by the program itself (slot stores).
    store_ranges: Vec<(u32, u32)>,
}

impl Tracer for CollisionTracer {
    fn event(&mut self, event: TraceEvent<'_>) {
        match event {
            TraceEvent::WindowSpill { sp, nregs, .. } => {
                self.spills += 1;
                self.spill_ranges.push((sp - 4 * nregs as u32, sp));
            }
            TraceEvent::WindowReload { .. } => self.reloads += 1,
            TraceEvent::MemWrite { addr, nbytes, .. } => {
                self.store_ranges.push((addr, addr + nbytes as u32));
            }
            _ => {}
        }
    }
}

impl CollisionTracer {
    /// First (spill_range, store_range) overlap, if any.
    fn collision(&self) -> Option<((u32, u32), (u32, u32))> {
        for &sp in &self.spill_ranges {
            for &st in &self.store_ranges {
                if sp.0 < st.1 && st.0 < sp.1 {
                    return Some((sp, st));
                }
            }
        }
        None
    }
}

fn inc_name(inc: CallInc) -> &'static str {
    match inc {
        CallInc::Call4 => "call4",
        CallInc::Call8 => "call8",
        CallInc::Call12 => "call12",
    }
}

/// P1-measured first-spill depth for this program shape (entry = the
/// recursive function), per increment.
fn spill_onset(inc: CallInc) -> u32 {
    match inc {
        CallInc::Call4 => 11,
        CallInc::Call8 => 6,
        CallInc::Call12 => 4,
    }
}

/// One matrix case: emulator run with the collision tracer (result, handler
/// activity, and address disjointness asserted — the tracer analysis is
/// profile-independent, so one traced run on the default profile suffices),
/// then the full N-run (every profile + every board) via the harness.
fn torture_case(h: &mut Harness, inc: CallInc, s_live: usize, pad: u32, depth: u32) {
    let name = format!("collide-{}-s{s_live}-p{pad}-d{depth}", inc_name(inc));
    let out = emit_program_with(&prog_rec_slots(s_live, pad), inc);
    assert!(out.sym_slots.is_empty());

    let mut emu = Emulator::new();
    let mut tr = CollisionTracer::default();
    let res = emu.run_traced(&out.code, out.entry_offset, depth, &mut tr);
    assert_eq!(res, EmuOutcome::Ok(depth), "[{name}] slot/register corruption");
    assert_eq!(tr.spills, tr.reloads, "[{name}] every spilled frame must reload");
    assert_eq!(
        tr.spills > 0,
        depth >= spill_onset(inc),
        "[{name}] spill activity vs measured onset {} (got {} spills)",
        spill_onset(inc),
        tr.spills
    );
    if let Some((sp, st)) = tr.collision() {
        panic!(
            "[{name}] COLLISION: window save-area write {sp:#x?} overlaps program \
             slot store {st:#x?}"
        );
    }
    eprintln!(
        "MEASURE collide inc={} slots={s_live} pad={pad} depth={depth} \
         spills={} reloads={} slot_stores={}",
        inc_name(inc),
        tr.spills,
        tr.reloads,
        tr.store_ranges.len()
    );

    h.nrun(&name, &out.code, out.entry_offset, depth, depth);
}

#[test]
fn spill_save_area_collision_torture() {
    let mut h = Harness::from_env(9500);

    // Primary matrix under the CALL8 policy: slot count x depth. Depths
    // straddle the measured overflow onset (6) and wrap the 64-register file
    // repeatedly (every 8 frames); slot counts run from minimal to a frame
    // 10x the save-area reservation.
    for s_live in [1usize, 2, 4, 8, 16, 64] {
        for depth in [1u32, 5, 8, 17, 33, 100] {
            torture_case(&mut h, CallInc::Call8, s_live, 0, depth);
        }
    }

    // Frame-size / placement variants: an unused pad slot below the live
    // slots moves them up against (and away from) the reserved top region.
    for (s_live, pad) in [(1usize, 100u32), (1, 244), (16, 100), (16, 244)] {
        for depth in [8u32, 100] {
            torture_case(&mut h, CallInc::Call8, s_live, pad, depth);
        }
    }

    // Call-increment variants: the save-area reservation rule must hold under
    // CALL4 (floored 32 B) and CALL12 (48 B) too.
    for inc in [CallInc::Call4, CallInc::Call12] {
        for s_live in [1usize, 16] {
            for depth in [8u32, 33, 100] {
                torture_case(&mut h, inc, s_live, 0, depth);
            }
        }
    }

    eprintln!(
        "spill_save_area_collision_torture: all cases passed (boards={})",
        h.boards.len()
    );
}
