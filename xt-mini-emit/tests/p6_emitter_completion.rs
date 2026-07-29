//! P6b — the remaining VInst variants (Load8U/8S/16U/16S, Neg, Bnot,
//! MemcpyWords), the large-frame limit, and the L32R full-range fix.
//!
//! Dual-run rig: emulator always; the real ESP32-S3 joins when
//! `XT_DEVICE_PORT` is set (single-threaded: `-- --test-threads=1`).
//! Emission-boundary tests (large frames, L32R displacement) are
//! emulator/emit-level by nature and run everywhere.

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::Emulator;

use xt_mini_emit::{
    emit_program, AluImmOp, AluOp, Callee, EmitOut, IcmpCond, MiniFunc, MiniProgram, MiniVInst,
    PReg,
};
use xt_runner_client::{RunOutcome as HwOutcome, Runner};

use MiniVInst::{
    AluRRI, AluRRR, Bnot, Br, BrIf, Call, IConst32, IcmpImm, Label, Load16S, Load16U, Load32,
    Load8S, Load8U, MemcpyWords, Neg, Ret, SlotAddr, Store32,
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
// 1. Sign/zero-extending loads.
// ---------------------------------------------------------------------------

/// Stores `a` and `!a` into an 8-byte slot, then reads bytes/halfwords back
/// with every extending-load flavor, folding with distinct weights so any
/// wrong extension or offset changes the result:
///
///   r  = l8ui  [w0+0]           (zero-extended byte)
///   r += l8si' [w0+1] * 3       (sign-extended byte: l8ui + sext)
///   r += l16ui [w0+2] * 5
///   r += l16si [w1+0] * 7
///   r += l8si' [w1+3] * 11
///   r += l16si [w0+0] * 13
fn prog_ext_loads() -> MiniProgram {
    let weighted_add = |insts: &mut Vec<MiniVInst>, w: i32| {
        // p5 = p4 * w; p6 += p5   (w materialized through p7)
        insts.push(IConst32 { dst: p(7), val: w });
        insts.push(AluRRR {
            op: AluOp::Mul,
            dst: p(5),
            src1: p(4),
            src2: p(7),
        });
        insts.push(AluRRR {
            op: AluOp::Add,
            dst: p(6),
            src1: p(6),
            src2: p(5),
        });
    };
    let mut insts = vec![
        SlotAddr { dst: p(3), slot: 0 },
        Store32 {
            src: p(2),
            base: p(3),
            offset: 0,
        },
        Bnot {
            dst: p(4),
            src: p(2),
        },
        Store32 {
            src: p(4),
            base: p(3),
            offset: 4,
        },
        // r (p6) = l8ui [0]
        Load8U {
            dst: p(6),
            base: p(3),
            offset: 0,
        },
    ];
    insts.push(Load8S {
        dst: p(4),
        base: p(3),
        offset: 1,
    });
    weighted_add(&mut insts, 3);
    insts.push(Load16U {
        dst: p(4),
        base: p(3),
        offset: 2,
    });
    weighted_add(&mut insts, 5);
    insts.push(Load16S {
        dst: p(4),
        base: p(3),
        offset: 4,
    });
    weighted_add(&mut insts, 7);
    insts.push(Load8S {
        dst: p(4),
        base: p(3),
        offset: 7,
    });
    weighted_add(&mut insts, 11);
    insts.push(Load16S {
        dst: p(4),
        base: p(3),
        offset: 0,
    });
    weighted_add(&mut insts, 13);
    insts.push(Ret { val: Some(p(6)) });
    MiniProgram {
        funcs: vec![func(vec![8], insts)],
    }
}

fn ext_loads_expect(a: u32) -> u32 {
    let w0 = a.to_le_bytes();
    let w1 = (!a).to_le_bytes();
    let mut r = w0[0] as u32; // l8ui [0]
    let mut add = |v: u32, w: u32| r = r.wrapping_add(v.wrapping_mul(w));
    add(w0[1] as i8 as i32 as u32, 3); // l8s [1]
    add(u16::from_le_bytes([w0[2], w0[3]]) as u32, 5); // l16ui [2]
    add(u16::from_le_bytes([w1[0], w1[1]]) as i16 as i32 as u32, 7); // l16si [4]
    add(w1[3] as i8 as i32 as u32, 11); // l8s [7]
    add(u16::from_le_bytes([w0[0], w0[1]]) as i16 as i32 as u32, 13); // l16si [0]
    r
}

// ---------------------------------------------------------------------------
// 2. Neg / Bnot.
// ---------------------------------------------------------------------------

/// f(a) = (-a) ^ (!a >> 1)
fn prog_neg_bnot() -> MiniProgram {
    prog1(vec![
        Neg {
            dst: p(3),
            src: p(2),
        },
        Bnot {
            dst: p(4),
            src: p(2),
        },
        AluRRI {
            op: AluImmOp::SrliU,
            dst: p(4),
            src: p(4),
            imm: 1,
        },
        AluRRR {
            op: AluOp::Xor,
            dst: p(2),
            src1: p(3),
            src2: p(4),
        },
        Ret { val: Some(p(2)) },
    ])
}

fn neg_bnot_expect(a: u32) -> u32 {
    a.wrapping_neg() ^ (!a >> 1)
}

// ---------------------------------------------------------------------------
// 3. MemcpyWords — including a copy past the 1020-byte offset ceiling
//    (chunked base-bumping) with base-register restoration verified.
// ---------------------------------------------------------------------------

const CPY_WORDS: u32 = 264; // 1056 bytes: 1024-byte chunk + 32-byte tail
const CPY_BYTES: u32 = CPY_WORDS * 4;
const CPY_K: i32 = 0x9E37; // pooled constant (outside movi range)

/// f(a): fill src[i] = (i * K) ^ a for i in 0..264; MemcpyWords dst <- src
/// (1056 bytes, two chunks); r = a + (src_base_after - src_base_before) +
/// (dst_base_after - dst_base_before), zero iff the emitter restored the
/// bases; then fold r = r*31 + dst[i] over the copy.
fn prog_memcpy() -> MiniProgram {
    let insts = vec![
        // ---- fill loop: p3 = moving ptr, p4 = i, p6 = K, p5 = v ----
        SlotAddr { dst: p(3), slot: 0 },
        IConst32 { dst: p(4), val: 0 },
        IConst32 {
            dst: p(6),
            val: CPY_K,
        },
        Label(0),
        IcmpImm {
            dst: p(5),
            src: p(4),
            imm: CPY_WORDS as i32,
            cond: IcmpCond::GeU,
        },
        BrIf {
            cond: p(5),
            target: 1,
            invert: false,
        },
        AluRRR {
            op: AluOp::Mul,
            dst: p(5),
            src1: p(4),
            src2: p(6),
        },
        AluRRR {
            op: AluOp::Xor,
            dst: p(5),
            src1: p(5),
            src2: p(2),
        },
        Store32 {
            src: p(5),
            base: p(3),
            offset: 0,
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(3),
            src: p(3),
            imm: 4,
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(4),
            src: p(4),
            imm: 1,
        },
        Br { target: 0 },
        Label(1),
        // ---- the copy, with fresh base registers ----
        SlotAddr { dst: p(3), slot: 0 },
        SlotAddr { dst: p(5), slot: 1 },
        MemcpyWords {
            dst_base: p(5),
            src_base: p(3),
            size: CPY_BYTES,
        },
        // ---- base-restoration check: p4 = (p3 - src) + (p5 - dst) ----
        SlotAddr { dst: p(7), slot: 0 },
        AluRRR {
            op: AluOp::Sub,
            dst: p(3),
            src1: p(3),
            src2: p(7),
        },
        SlotAddr { dst: p(7), slot: 1 },
        AluRRR {
            op: AluOp::Sub,
            dst: p(5),
            src1: p(5),
            src2: p(7),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(4),
            src1: p(3),
            src2: p(5),
        },
        // ---- fold loop over dst: r = p3 = a + basediff; p5 = ptr, p4 = i,
        //      p6 = 31, p7 = scratch ----
        AluRRR {
            op: AluOp::Add,
            dst: p(3),
            src1: p(2),
            src2: p(4),
        },
        SlotAddr { dst: p(5), slot: 1 },
        IConst32 { dst: p(4), val: 0 },
        IConst32 { dst: p(6), val: 31 },
        Label(2),
        IcmpImm {
            dst: p(7),
            src: p(4),
            imm: CPY_WORDS as i32,
            cond: IcmpCond::GeU,
        },
        BrIf {
            cond: p(7),
            target: 3,
            invert: false,
        },
        Load32 {
            dst: p(7),
            base: p(5),
            offset: 0,
        },
        AluRRR {
            op: AluOp::Mul,
            dst: p(3),
            src1: p(3),
            src2: p(6),
        },
        AluRRR {
            op: AluOp::Add,
            dst: p(3),
            src1: p(3),
            src2: p(7),
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(5),
            src: p(5),
            imm: 4,
        },
        AluRRI {
            op: AluImmOp::Addi,
            dst: p(4),
            src: p(4),
            imm: 1,
        },
        Br { target: 2 },
        Label(3),
        Ret { val: Some(p(3)) },
    ];
    MiniProgram {
        funcs: vec![func(vec![CPY_BYTES, CPY_BYTES], insts)],
    }
}

fn memcpy_expect(a: u32) -> u32 {
    let mut r = a; // + 0 base diff
    for i in 0..CPY_WORDS {
        let v = i.wrapping_mul(CPY_K as u32) ^ a;
        r = r.wrapping_mul(31).wrapping_add(v);
    }
    r
}

// ---------------------------------------------------------------------------
// 4. Large frames — a 2 KiB-frame function dual-runs; the ENTRY boundary is
//    emission-level (below).
// ---------------------------------------------------------------------------

/// f(a): slot of 2048 bytes; store a at the first and last word, read both
/// back through the >addi-range offsets. Frame = 2080.
fn prog_frame_2k() -> MiniProgram {
    MiniProgram {
        funcs: vec![func(
            vec![2048],
            vec![
                SlotAddr { dst: p(3), slot: 0 },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: 0,
                },
                Bnot {
                    dst: p(4),
                    src: p(2),
                },
                Store32 {
                    src: p(4),
                    base: p(3),
                    offset: 2044, // > 1020: goes through the AddressScratch path
                },
                Load32 {
                    dst: p(5),
                    base: p(3),
                    offset: 2044,
                },
                Load32 {
                    dst: p(6),
                    base: p(3),
                    offset: 0,
                },
                AluRRR {
                    op: AluOp::Xor,
                    dst: p(2),
                    src1: p(5),
                    src2: p(6),
                },
                Ret { val: Some(p(2)) },
            ],
        )],
    }
}

fn frame_2k_expect(a: u32) -> u32 {
    !a ^ a
}

// ---------------------------------------------------------------------------
// The single dual-run test (device cases must not run concurrently).
// ---------------------------------------------------------------------------

#[test]
fn p6_corpus_dual_run() {
    let mut dev = device();
    let mut seq = 8000u32;
    let args = [
        0u32,
        1,
        0x7F,
        0x80,
        0xFF,
        0x8081_7FFE,
        0xFFFF_FFFF,
        0x1234_ABCD,
    ];

    let loads = emit_program(&prog_ext_loads());
    for a in args {
        dual_run(
            &mut dev,
            &mut seq,
            "p6-ext-loads",
            &loads,
            a,
            ext_loads_expect(a),
        );
    }

    let negbnot = emit_program(&prog_neg_bnot());
    for a in args {
        dual_run(
            &mut dev,
            &mut seq,
            "p6-neg-bnot",
            &negbnot,
            a,
            neg_bnot_expect(a),
        );
    }

    let memcpy = emit_program(&prog_memcpy());
    for a in [0u32, 1, 0xDEAD_BEEF] {
        dual_run(
            &mut dev,
            &mut seq,
            "p6-memcpy",
            &memcpy,
            a,
            memcpy_expect(a),
        );
    }

    let frame2k = emit_program(&prog_frame_2k());
    for a in [0u32, 42, 0x8000_0000] {
        dual_run(
            &mut dev,
            &mut seq,
            "p6-frame-2k",
            &frame2k,
            a,
            frame_2k_expect(a),
        );
    }

    eprintln!(
        "p6_corpus_dual_run: all cases passed (device={})",
        dev.is_some()
    );
}

// ---------------------------------------------------------------------------
// Emulator-only: MemcpyWords degenerate cases.
// ---------------------------------------------------------------------------

#[test]
fn p6_memcpy_degenerate_emu() {
    // size 0 and self-copy are no-ops: f(a) = a survives both.
    let prog = MiniProgram {
        funcs: vec![func(
            vec![16, 16],
            vec![
                SlotAddr { dst: p(3), slot: 0 },
                SlotAddr { dst: p(4), slot: 1 },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: 0,
                },
                MemcpyWords {
                    dst_base: p(4),
                    src_base: p(3),
                    size: 0,
                },
                MemcpyWords {
                    dst_base: p(3),
                    src_base: p(3),
                    size: 16,
                },
                Load32 {
                    dst: p(2),
                    base: p(3),
                    offset: 0,
                },
                Ret { val: Some(p(2)) },
            ],
        )],
    };
    let out = emit_program(&prog);
    for a in [0u32, 7, 0xFFFF_FFFF] {
        match emu_run(&out.code, out.entry_offset, a) {
            EmuOutcome::Ok(v) => assert_eq!(v, a),
            other => panic!("degenerate memcpy a={a}: {other:?}"),
        }
    }

    // A non-multiple-of-4 size is an emit-time invariant violation.
    let bad = prog1(vec![
        MemcpyWords {
            dst_base: p(3),
            src_base: p(4),
            size: 6,
        },
        Ret { val: None },
    ]);
    assert!(
        std::panic::catch_unwind(|| emit_program(&bad)).is_err(),
        "MemcpyWords with a non-word size must refuse to emit"
    );
}

// ---------------------------------------------------------------------------
// The large-frame limit: ENTRY's immediate is 0..=32760 step 8; frames are
// rounded to STACK_ALIGNMENT (16), so the largest emittable frame is 32752 —
// slot bytes <= 32720 under the 32-byte save-area reservation. One slot-word
// past that must be the DOCUMENTED HARD ERROR (never `lp_xt_inst::encode`'s
// silent truncation, which would produce `entry a1, 0`).
// ---------------------------------------------------------------------------

/// A function whose single slot is `slot_bytes` and which round-trips `a`
/// through the slot's first and last words.
fn frame_prog(slot_bytes: u32) -> MiniProgram {
    let last = (slot_bytes - 4) as i32;
    MiniProgram {
        funcs: vec![func(
            vec![slot_bytes],
            vec![
                SlotAddr { dst: p(3), slot: 0 },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: 0,
                },
                Store32 {
                    src: p(2),
                    base: p(3),
                    offset: last,
                },
                Load32 {
                    dst: p(4),
                    base: p(3),
                    offset: last,
                },
                Ret { val: Some(p(4)) },
            ],
        )],
    }
}

#[test]
fn p6_entry_frame_boundary() {
    // Exactly the ceiling: slot 32720 -> frame 32752 (<= 32760, 16-aligned).
    let out = emit_program(&frame_prog(32720));
    let (inst, _) = lp_xt_inst::decode(&out.code[out.entry_offset as usize..]).unwrap();
    match inst {
        lp_xt_inst::Inst::Entry(_, frame) => assert_eq!(frame, 32752),
        other => panic!("expected ENTRY first, got {other:?}"),
    }
    // The max-frame function runs on the emulator (128 KiB stack region).
    // NOTE for the hardware pass: verify the device runner's task stack can
    // absorb a 32 KiB payload frame before adding this to a device corpus.
    for a in [0u32, 0xA5A5_5A5A] {
        match emu_run(&out.code, out.entry_offset, a) {
            EmuOutcome::Ok(v) => assert_eq!(v, a),
            other => panic!("max-frame run a={a}: {other:?}"),
        }
    }

    // One slot-word past the ceiling: frame would round to 32768 > 32760 —
    // must refuse with the documented error, not truncate.
    let err = std::panic::catch_unwind(|| emit_program(&frame_prog(32724)))
        .expect_err("frame past ENTRY's immediate must refuse to emit");
    let msg = err
        .downcast_ref::<String>()
        .cloned()
        .unwrap_or_else(|| err.downcast_ref::<&str>().unwrap_or(&"").to_string());
    assert!(
        msg.contains("32760") && msg.contains("MOVSP"),
        "error must name the ENTRY limit and the policy (got: {msg})"
    );
}

// ---------------------------------------------------------------------------
// L32R full-range fix. The 16-bit field is ONE-extended (value = field -
// 0x10000): displacements -262144..=-4 are legal, and encoded fields
// 0x0000..0x7FFF reach the FARTHER 128 KiB — the emitter's old assert
// (-32768..0 on the sign-extended field) refused exactly that half.
// Layout arithmetic for a 1-literal program with n 3-byte filler insts:
// entry at buffer offset 4, the l32r at pc = 7 + 3n, its base =
// ((pc + 3) & !3), displacement = -base (pool slot 0).
// ---------------------------------------------------------------------------

/// One pooled-constant load preceded by `n` 3-byte filler instructions:
/// f(a) = a + 0x12345678, with the l32r `n` instructions from the pool.
fn l32r_prog(n: usize) -> MiniProgram {
    let mut insts = Vec::with_capacity(n + 3);
    for _ in 0..n {
        insts.push(AluRRI {
            op: AluImmOp::Addi,
            dst: p(3),
            src: p(3),
            imm: 1,
        });
    }
    insts.push(IConst32 {
        dst: p(4),
        val: 0x1234_5678,
    });
    insts.push(AluRRR {
        op: AluOp::Add,
        dst: p(2),
        src1: p(2),
        src2: p(4),
    });
    insts.push(Ret { val: Some(p(2)) });
    prog1(insts)
}

/// Decode the l32r at its computed offset and return its raw 16-bit field.
fn l32r_field(out: &EmitOut, n: usize) -> u16 {
    let pc = 7 + 3 * n;
    let (inst, _) = lp_xt_inst::decode(&out.code[pc..]).expect("l32r site must decode");
    match inst {
        lp_xt_inst::Inst::L32r(_, field) => field,
        other => panic!("expected l32r at offset {pc}, got {other:?}"),
    }
}

#[test]
fn p6_l32r_displacement_boundaries() {
    // Crossover: disp = -131076 (first value past the sign-extended-i16
    // half). Old assert: panic. New rule: legal, field = 0x7FFF.
    // pc = 7 + 3*43689 = 131074, base = 131076.
    let out = emit_program(&l32r_prog(43689));
    assert_eq!(l32r_field(&out, 43689), 0x7FFF);

    // Last value inside the old half: disp = -131072, field = 0x8000
    // (legal under both rules; pins that the near half still encodes as the
    // sign-extended view).
    let out = emit_program(&l32r_prog(43688));
    assert_eq!(l32r_field(&out, 43688), 0x8000);

    // The true far boundary: disp = -262144, field = 0x0000.
    // pc = 7 + 3*87378 = 262141, base = 262144.
    let out = emit_program(&l32r_prog(87378));
    assert_eq!(l32r_field(&out, 87378), 0x0000);

    // One step beyond: disp = -262148 — no encoding; must refuse.
    assert!(
        std::panic::catch_unwind(|| emit_program(&l32r_prog(87380))).is_err(),
        "L32R displacement past -262144 must refuse to emit"
    );
}

/// End-to-end: a pooled literal ~127 KiB behind the load executes correctly
/// (deepest reach that still fits the emulator's 128 KiB code region; the
/// far half 0x0000..0x7FFF is emission-verified above and — like everything
/// else — belongs to the hardware pass, since the emulator currently
/// sign-extends the field).
#[test]
fn p6_l32r_deep_reach_emu() {
    let n = 43300; // code ≈ 130 KB total, disp ≈ -130 KiB
    let out = emit_program(&l32r_prog(n));
    assert!(out.code.len() <= lp_xt_emu::emu::CODE_REGION_LEN);
    for a in [0u32, 5] {
        match emu_run(&out.code, out.entry_offset, a) {
            EmuOutcome::Ok(v) => assert_eq!(v, a.wrapping_add(0x1234_5678)),
            other => panic!("deep l32r a={a}: {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Position-independence guard: nothing above uses Callee::Sym, and the Call
// variant still refuses stack args (pinned elsewhere); this file's programs
// must all stay dual-runnable.
// ---------------------------------------------------------------------------

#[test]
fn p6_programs_are_position_independent() {
    for prog in [
        prog_ext_loads(),
        prog_neg_bnot(),
        prog_memcpy(),
        prog_frame_2k(),
    ] {
        assert!(emit_program(&prog).sym_slots.is_empty());
    }
    // Local-function calls still work alongside the new variants.
    let out = emit_program(&MiniProgram {
        funcs: vec![
            func(
                vec![],
                vec![
                    Call {
                        callee: Callee::Func(1),
                        args: vec![p(2)],
                        ret: Some(p(2)),
                    },
                    Ret { val: Some(p(2)) },
                ],
            ),
            func(
                vec![],
                vec![
                    Neg {
                        dst: p(2),
                        src: p(2),
                    },
                    Ret { val: Some(p(2)) },
                ],
            ),
        ],
    });
    match emu_run(&out.code, out.entry_offset, 5) {
        EmuOutcome::Ok(v) => assert_eq!(v, 5u32.wrapping_neg()),
        other => panic!("neg-through-call: {other:?}"),
    }
}
