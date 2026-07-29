//! `xt-mini-emit` — prototype MiniVInst -> Xtensa (ESP32-S3) code emitter.
//!
//! [`MiniVInst`] is a thin structural mirror of lightplayer's
//! `lpvm-native::vinst::VInst` at the emit stage (operands are pre-allocated
//! physical registers), so every hard emitter sub-problem solved here —
//! literal pools, branch fixups/relaxation, `ENTRY` frames, the windowed call
//! ABI — ports into `lpvm-native/src/isa/xt/emit.rs` by mechanical
//! substitution. See `README.md` for the MiniVInst<->VInst mapping table and
//! the emitter policy (pool-before-code, backward `L32R`, wide forms only).
//!
//! All machine encodings come from [`lp_xt_inst::encode`] — this crate never
//! writes instruction bytes by hand. Emitted programs are validated by
//! dual-running on `lp-xt-emu` and, when `XT_DEVICE_PORT` is set, on a real
//! ESP32-S3 via `xt-runner-client` (`tests/dual_run.rs`).
//!
//! This crate is original code; see the README Provenance section and
//! `docs/adr/2026-07-28-license-provenance-discipline.md`.

#![no_std]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod emit;
pub mod imm;
pub mod vinst;

pub use emit::{emit_program, EmitOut};
pub use vinst::{
    AluImmOp, AluOp, Callee, IcmpCond, LabelId, MiniFunc, MiniProgram, MiniVInst, PReg, SymbolId,
};

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    fn p(n: u8) -> PReg {
        PReg(n)
    }

    fn prog1(insts: Vec<MiniVInst>) -> MiniProgram {
        MiniProgram {
            funcs: vec![MiniFunc {
                slots: vec![],
                insts,
            }],
        }
    }

    /// The trivial constant function must reproduce the spike's GV1 golden
    /// vector byte-for-byte (`entry a1,32; movi a2,42; retw`, objdump-derived
    /// — see FINDINGS.md).
    #[test]
    fn gv1_stub42_exact_bytes() {
        let out = emit_program(&prog1(vec![
            MiniVInst::IConst32 { dst: p(2), val: 42 },
            MiniVInst::Ret { val: Some(p(2)) },
        ]));
        assert_eq!(out.entry_offset, 0);
        assert_eq!(
            out.code,
            vec![0x36, 0x41, 0x00, 0x22, 0xa0, 0x2a, 0x90, 0x00, 0x00]
        );
    }

    /// Large constants are pooled before the code and deduplicated; the pool
    /// pushes the entry offset up and every L32R points backward.
    #[test]
    fn literal_pool_dedup_and_layout() {
        let out = emit_program(&prog1(vec![
            MiniVInst::IConst32 {
                dst: p(3),
                val: 0x12345678,
            },
            MiniVInst::IConst32 {
                dst: p(4),
                val: 0x12345678,
            },
            MiniVInst::IConst32 {
                dst: p(5),
                val: -559038737, // 0xDEADBEEF
            },
            MiniVInst::Ret { val: Some(p(3)) },
        ]));
        // Two distinct literals -> 8-byte pool, entry right after.
        assert_eq!(out.entry_offset, 8);
        assert_eq!(&out.code[0..4], &0x12345678u32.to_le_bytes());
        assert_eq!(&out.code[4..8], &0xDEADBEEFu32.to_le_bytes());
        // Three l32r instructions (one per IConst32 — dedup collapses pool
        // slots, not loads), all decodable at their sites.
        let n_l32r = decode_all(&out.code[out.entry_offset as usize..])
            .iter()
            .filter(|i| matches!(i, lp_xt_inst::Inst::L32r(..)))
            .count();
        assert_eq!(n_l32r, 3);
    }

    /// Every non-entry function entry is 4-byte aligned (CALL8 targets are
    /// computed `(PC & !3) + (off << 2) + 4`).
    #[test]
    fn function_entries_are_aligned() {
        let callee = MiniFunc {
            slots: vec![],
            insts: vec![
                MiniVInst::AluRRI {
                    op: AluImmOp::Addi,
                    dst: p(2),
                    src: p(2),
                    imm: 1,
                },
                MiniVInst::Ret { val: Some(p(2)) },
            ],
        };
        let main = MiniFunc {
            slots: vec![],
            insts: vec![
                MiniVInst::Call {
                    callee: Callee::Func(1),
                    args: vec![p(2)],
                    ret: Some(p(3)),
                },
                MiniVInst::Ret { val: Some(p(3)) },
            ],
        };
        let out = emit_program(&MiniProgram {
            funcs: vec![main, callee],
        });
        for off in &out.func_offsets {
            assert_eq!(off % 4, 0, "function entry {off} not 4-aligned");
        }
    }

    /// A conditional branch across > 2 KB of code must relax to the inverted
    /// beqz/bnez over a `J`.
    #[test]
    fn long_conditional_branch_relaxes() {
        let mut insts = vec![MiniVInst::BrIf {
            cond: p(2),
            target: 0,
            invert: false,
        }];
        for _ in 0..900 {
            insts.push(MiniVInst::AluRRI {
                op: AluImmOp::Addi,
                dst: p(3),
                src: p(3),
                imm: 1,
            });
        }
        insts.push(MiniVInst::Label(0));
        insts.push(MiniVInst::Ret { val: Some(p(3)) });
        let out = emit_program(&prog1(insts));
        // The relaxed sequence is the inverted beqz over a J, right after ENTRY.
        let decoded = decode_all(&out.code);
        assert!(
            matches!(
                decoded[1],
                lp_xt_inst::Inst::BranchZ(lp_xt_inst::BrZ::Beqz, _, 2)
            ),
            "expected inverted beqz over J, got {:?}",
            decoded[1]
        );
        assert!(
            matches!(decoded[2], lp_xt_inst::Inst::J(_)),
            "expected J after inverted branch, got {:?}",
            decoded[2]
        );
    }

    /// Callee::Sym pools one patchable slot per symbol and reports its offset.
    #[test]
    fn sym_call_reports_pool_slot() {
        let out = emit_program(&prog1(vec![
            MiniVInst::Call {
                callee: Callee::Sym(SymbolId(7)),
                args: vec![p(2)],
                ret: Some(p(2)),
            },
            MiniVInst::Call {
                callee: Callee::Sym(SymbolId(7)),
                args: vec![p(2)],
                ret: Some(p(2)),
            },
            MiniVInst::Ret { val: Some(p(2)) },
        ]));
        // One slot (deduped), at pool offset 0; entry after the 4-byte pool.
        assert_eq!(out.sym_slots, vec![(SymbolId(7), 0)]);
        assert_eq!(out.entry_offset, 4);
    }

    /// Decode the emitted stream back through lp-xt-inst (sanity: everything
    /// we emit round-trips the shared encoder/decoder).
    fn decode_all(mut bytes: &[u8]) -> Vec<lp_xt_inst::Inst> {
        let mut out = Vec::new();
        while !bytes.is_empty() {
            let (inst, len) = lp_xt_inst::decode(bytes).expect("emitted bytes must decode");
            out.push(inst);
            bytes = &bytes[len..];
        }
        out
    }
}
