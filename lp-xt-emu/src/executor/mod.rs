//! Per-group instruction executors, mirroring the lp-riscv-emu split
//! (arith / imm / load_store / branch / jump / call / window / misc). Each
//! module is an `impl Emulator` block; this file only routes a decoded
//! [`Inst`] to the right group.
//!
//! Semantics come from the Xtensa ISA Reference Manual; no QEMU/binutils source
//! was used (see the repo license ADR).

use lp_xt_inst::{Inst, NullaryNarrowOp, NullaryOp};

use crate::emu::{Emulator, Flow};
use crate::error::Trap;
use crate::trace::Tracer;

mod arith;
mod branch;
mod call;
mod imm;
mod jump;
mod load_store;
mod misc;
mod window;

impl Emulator {
    /// Execute one decoded instruction. `pc`/`len` describe the current
    /// instruction; the returned [`Flow`] tells the run loop how to advance.
    pub(crate) fn execute(
        &mut self,
        inst: &Inst,
        pc: u32,
        tracer: &mut dyn Tracer,
    ) -> Result<Flow, Trap> {
        match inst {
            // --- arithmetic / logical / shift (register + register-immediate shifts) ---
            Inst::Rrr(..)
            | Inst::Rt(..)
            | Inst::Rs(..)
            | Inst::ShiftSet(..)
            | Inst::Ssai(..)
            | Inst::Slli(..)
            | Inst::Srli(..)
            | Inst::Srai(..)
            | Inst::Extui(..)
            | Inst::Sext(..)
            | Inst::AddN(..)
            | Inst::MovN(..) => self.exec_arith(inst, tracer),

            // --- immediate / move ---
            Inst::Movi(..)
            | Inst::MoviN(..)
            | Inst::Addi(..)
            | Inst::AddiN(..)
            | Inst::Addmi(..) => self.exec_imm(inst, tracer),

            // --- load / store (incl. l32r literal load) ---
            Inst::Load(..)
            | Inst::Store(..)
            | Inst::L32iN(..)
            | Inst::S32iN(..)
            | Inst::L32r(..) => self.exec_load_store(inst, pc, tracer),

            // --- conditional branches ---
            Inst::BranchRr(..)
            | Inst::BranchRi(..)
            | Inst::BranchRiu(..)
            | Inst::BranchZ(..)
            | Inst::BranchBiI(..)
            | Inst::BranchZN(..) => self.exec_branch(inst, pc),

            // --- unconditional jumps ---
            Inst::J(..) | Inst::Jx(..) => self.exec_jump(inst, pc),

            // --- calls ---
            Inst::Call(..) | Inst::Callx(..) => self.exec_call(inst, pc, tracer),

            // --- window management ---
            Inst::Entry(..) => self.exec_entry(inst, tracer),
            Inst::Nullary(NullaryOp::Retw) | Inst::NullaryN(NullaryNarrowOp::RetwN) => {
                self.exec_retw(tracer)
            }
            Inst::Nullary(NullaryOp::Ret) | Inst::NullaryN(NullaryNarrowOp::RetN) => {
                Ok(self.exec_ret())
            }

            // --- misc / barriers / nops / illegal ---
            Inst::Nullary(_) | Inst::NullaryN(_) => self.exec_misc(inst, pc),
        }
    }
}
