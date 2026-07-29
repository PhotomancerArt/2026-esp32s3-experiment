//! Xtensa windowed-ABI constants for the lp2025 backport — the `abi.rs` half
//! of the contract module (see `gpr.rs` for the register model).
//!
//! Shape-for-shape mirror of the constants `lpvm-native/src/isa/rv32/abi.rs`
//! defines (`SRET_SCALAR_THRESHOLD`, `STACK_ALIGNMENT`), destined for
//! `lpvm-native/src/isa/xt/abi.rs`, plus the **one genuine ISA hook Xtensa
//! adds**: [`FRAME_TOP_RESERVED_BYTES`], the window save-area reservation
//! that `abi/frame.rs::compute()` must place at the top of every frame
//! (rv32 = 0). The rv32 original's classification *functions*
//! (`classify_params` / `classify_return` / `func_abi_*`) depend on
//! lpvm-native types and stay in the monorepo; the register facts they need
//! are all here and in `gpr.rs`.
//!
//! Everything is pinned by `docs/adr/2026-07-28-xtensa-abi-contract.md`,
//! backed by the P1 silicon measurements (`docs/call-inc-study.md`).
//!
//! ## LX6 (classic ESP32) note
//!
//! No value in this module differs on LX6: the windowed ABI's alignment,
//! save-area layout, and CALL8 semantics are identical there.

use crate::emit::CallInc;

/// The windowed call increment — **CALL8**, the P2 policy decision, measured
/// in the P1 study (`docs/call-inc-study.md`):
///
/// - CALL4: only 2 registers survive a call (spill around *every* call) and
///   its arg staging overlaps program regs `a6/a7` (a permanent emitter
///   special case). Rejected.
/// - CALL12: only **2** register args — 3-arg calls cannot be emitted at all
///   (pinned by test); LPIR calls routinely carry 3+. Rejected.
/// - CALL8: 6 preserved temporaries, 6 register args, conflict-free staging,
///   32-byte frame reservation. No disqualifier.
///
/// This constant is the single source of truth: `CallInc::default()` and
/// [`FRAME_TOP_RESERVED_BYTES`] derive from it (asserted in tests).
pub const CALL_INC: CallInc = CallInc::Call8;

/// Reserved bytes at the **top** of every frame for the windowed ABI's
/// register save areas — **the ISA hook `abi/frame.rs::compute()` needs**
/// (rv32 = 0; Xtensa/CALL8 = 32).
///
/// The window overflow/underflow trap handlers write/read these bytes
/// *unbidden*, whenever call depth exceeds the physical register file: a
/// CALL8-entered frame owes the 16-byte base save area (an ancestor's
/// `a0..a3` land at `[SP-16, SP)` of its *callee*) plus 16 bytes for the
/// `a4..a7` group spilled by `_WindowOverflow8` — `16 * units` total
/// ([`CallInc::save_area_bytes`], derived from the handler contract and
/// hardware-validated by the M5/P1 recursion corpus: spill/reload
/// round-trips correct on silicon at every depth 1..=40, and slotted
/// recursion to depth 100 proves stack slots never collide with this
/// region). Frame layout: this reservation at the frame TOP; slots, spills,
/// and outgoing stack args build upward from `SP+0` exactly as rv32 does.
/// Getting this wrong corrupts *ancestor* frames invisibly — hence P5's
/// torture suite.
pub const FRAME_TOP_RESERVED_BYTES: u32 = 32;

/// Direct-return width: more than 2 scalar return words go through an sret
/// buffer. Same value as rv32 — deliberately:
///
/// - **Contract parity.** rv32 chose 2 to match Cranelift's
///   `signature_for_ir_func`; keeping the same threshold makes LPIR-level
///   return classification (vec2 direct; vec3/vec4 sret) target-invariant,
///   so shared filetests and call-lowering behave identically on both ISAs.
/// - **Stays inside the proven register contract.** Two return words are
///   callee `a2,a3` -> caller `a10,a11` ([`crate::gpr::RET_REGS`] /
///   [`crate::gpr::CALL_RET_REGS`]), the same registers the measured CALL8
///   staging already exercises. The windowed ABI would permit up to 4 direct
///   return words (`a2..a5`), but widening the contract buys nothing LPIR
///   needs and diverges classification across targets. P4 validates the
///   2-word direct + sret paths on hardware.
pub const SRET_SCALAR_THRESHOLD: usize = 2;

/// Stack-pointer alignment, in bytes. The Xtensa windowed ABI mandates
/// 16-byte SP alignment (and the save-area layout assumes it: the 16-byte
/// base save area sits at `[SP-16, SP)`); `ENTRY`'s frame immediate is
/// coarser-grained (multiples of 8) so the emitter rounds frames up to this.
/// Same value as rv32, for the ABI's own reasons — not copied.
pub const STACK_ALIGNMENT: u32 = 16;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpr;

    /// The frame reservation is exactly what the chosen increment's handler
    /// contract requires — one value, derived, not free-floating.
    #[test]
    fn frame_reservation_matches_policy() {
        assert_eq!(FRAME_TOP_RESERVED_BYTES, CALL_INC.save_area_bytes());
        assert_eq!(FRAME_TOP_RESERVED_BYTES % STACK_ALIGNMENT, 0);
    }

    /// `CallInc::default()` (what `emit_program` uses) is the policy constant.
    #[test]
    fn default_call_inc_is_the_policy() {
        assert_eq!(CallInc::default(), CALL_INC);
    }

    /// Direct returns fit the return-register pairs in both views.
    #[test]
    fn sret_threshold_fits_ret_regs() {
        assert_eq!(SRET_SCALAR_THRESHOLD, gpr::RET_REGS.len());
        assert_eq!(SRET_SCALAR_THRESHOLD, gpr::CALL_RET_REGS.len());
    }
}
