//! GPR index helpers for Xtensa (ESP32-S3 / LX7) emission (`PReg` = `u8`,
//! a0–a15) — the hardware-validated register model for the lp2025 backport.
//!
//! Shape-for-shape mirror of `lpvm-native/src/isa/rv32/gpr.rs`, destined for
//! `lpvm-native/src/isa/xt/gpr.rs`. Values encode the **CALL8 windowed
//! calling convention** measured on silicon in the P1 call-increment study
//! ([`docs/call-inc-study.md`](../docs/call-inc-study.md)) and pinned by
//! `docs/adr/2026-07-28-xtensa-abi-contract.md`.
//!
//! The one Xtensa-specific wrinkle rv32 does not have: a call *rotates* the
//! register window, so the caller and callee see different names for the same
//! physical registers. Every constant below states which view it is in.
//! The two views are linked by [`CALL_ROTATION`]:
//! `caller a[n + CALL_ROTATION]` == `callee a[n]`.
//!
//! Note on naming: [`PReg`] here is the raw hardware register number (the
//! rv32 shape); `crate::vinst::PReg` is the newtype MiniVInst carries as an
//! operand. The backport keeps only this one.
//!
//! ## LX6 (classic ESP32) note — verified, no longer asserted
//!
//! Nothing in this module is LX7-specific: the windowed ABI, the 16-register
//! window over a 64-entry file, and CALL8/ENTRY/RETW semantics are identical
//! on LX6. Every constant carries over to classic ESP32 unchanged.
//!
//! Evidence (2026-07-28): the C3/C4 ladder ran the LX7-assembled golden
//! vectors byte-for-byte on classic silicon (windowed CALLX8, depth-100
//! recursion, first spill ~depth 6 per the CALL8 model), and the P5 N-run
//! corpus — including the call-boundary torture that pins the preserved set
//! at exactly `a2..a7` and the spill/reload paths at depth — passed on a
//! classic ESP32 v3 with zero divergences (FINDINGS.md, "LX6 conformance").

/// Physical GPR index (a0–a15).
pub type PReg = u8;

/// `a0` — return address (written by `CALLn`, mangled with the CALLINC bits;
/// `RETW` consumes it). Never allocatable.
pub const RA_REG: PReg = 0;

/// `a1` — stack pointer. `ENTRY a1, frame` derives the callee SP from the
/// caller's; stable for the whole frame. Never allocatable.
pub const SP_REG: PReg = 1;

/// Frame pointer: **Xtensa needs none — aliased to [`SP_REG`]**.
///
/// rv32 dedicates s0 as a frame pointer because its prologue adjusts SP and
/// large/dynamic frames want a fixed base. Under the windowed ABI neither
/// reason exists: `ENTRY` establishes the frame in one instruction and `a1`
/// is then invariant for the frame's lifetime (frames are fixed-size — the
/// emitter hard-errors past `ENTRY`'s 32760-byte immediate rather than
/// emitting the `movsp` dynamic-stack idiom, and LPIR has no alloca). All
/// slot/spill addressing is SP-relative. Aliasing keeps the rv32 shape
/// without burning one of only 16 window registers on a register with no job.
pub const FP_REG: PReg = SP_REG;

/// Window rotation of a `CALL8`, in registers: `caller a[n + 8]` is the same
/// physical register as `callee a[n]`. This is the number that converts
/// between the caller-view and callee-view constants below
/// (`4 * CallInc::Call8.units()`; asserted in tests).
pub const CALL_ROTATION: u8 = 8;

/// Incoming argument registers, **callee view**: parameters arrive in the
/// callee's `a2..=a7` (staged by the caller at `a10..=a15` = [`OUT_ARG_REGS`]
/// pre-rotation). This is the view register allocation uses — param vregs
/// precolor here, exactly like rv32's `ARG_REGS` (which has 8; the windowed
/// ABI caps register args at 6 for every increment — measured, P1 §1).
pub const ARG_REGS: [PReg; 6] = [2, 3, 4, 5, 6, 7];

/// Outgoing argument staging registers, **caller view**: the emitter writes
/// call arguments to its `a10..=a15`, which the callee's `ENTRY` rotates into
/// its `a2..=a7`. `OUT_ARG_REGS[i] == ARG_REGS[i] + CALL_ROTATION` (asserted).
/// Disjoint from [`ARG_REGS`] — the CALL8 staging area never aliases the
/// preserved bank, so argument moves need no parallel-move resolution
/// (CALL4's staging overlaps `a6/a7`; one of the reasons it lost, P1 §1).
pub const OUT_ARG_REGS: [PReg; 6] = [10, 11, 12, 13, 14, 15];

/// Return-value registers, **callee view**: `Ret` writes `a2` (`a3` for a
/// second word). Mirrors rv32's 2-word direct-return contract
/// (`SRET_SCALAR_THRESHOLD` — see `abi.rs`).
pub const RET_REGS: [PReg; 2] = [2, 3];

/// Return-value registers, **caller view**: after the call returns, the
/// callee's `a2, a3` are the caller's `a10, a11`
/// (`CALL_RET_REGS[i] == RET_REGS[i] + CALL_ROTATION`; asserted). This is
/// where the emitter reads a call's result.
pub const CALL_RET_REGS: [PReg; 2] = [10, 11];

/// Primary emitter scratch (`a8`) for lowering sequences — icmp/select
/// staging, out-of-range address arithmetic, `callx8` targets. NOT in
/// [`ALLOC_POOL`] (rv32 shape: `SCRATCH` = t3, plus emitter TEMP0–2).
///
/// Why `a8`/`a9`: they are the only caller-saved registers that are *not*
/// argument staging — `a8` is where `CALL8` writes the (mangled) return
/// address and `a9` falls in the same dead zone below the staging area, so
/// nothing can be live there across a call anyway. Zero-cost scratch.
pub const SCRATCH: PReg = 8;

/// Secondary emitter scratch (`a9`); same rationale as [`SCRATCH`].
pub const SCRATCH2: PReg = 9;

/// Registers available to the allocator for temporaries — **12** (vs rv32's
/// 13: near parity, the windowed file does not halve the pool).
///
/// Contents: everything except a0/a1 (RA/SP) and a8/a9 (emitter scratch).
/// Unlike rv32, the incoming-argument registers ARE in the pool: on rv32 the
/// arg registers are also the *outgoing* staging area of every call, so
/// pooling them would put each call in conflict with every live temporary;
/// under CALL8 the outgoing staging is the *separate* caller-saved bank
/// `a10..=a15`, and the callee's `a2..=a7` are, after the precolored
/// parameters die, ordinary call-preserved temporaries. A 16-register window
/// cannot afford an unpooled 6-register arg bank, and does not need one.
///
/// Order = the allocator's LRU initialization order (rv32 front-loads its
/// caller-saved t4–t6 so short-lived values land where a call clobbers them
/// for free; same policy here):
///
/// - **Caller-saved first**, `a15` down to `a10`: descending, because
///   outgoing arguments stage upward from `a10` — handing out `a15` first
///   keeps the low staging slots (used by *every* call; most calls have few
///   args) free longest.
/// - **Preserved bank next**, `a7` down to `a2`: descending, because `a2`/
///   `a3` are the return registers and first parameters — keeping them free
///   longest makes the `mov RET_REGS[0], val` before `retw` (and param
///   staying-in-place) a no-op more often.
///
/// Preservation across calls is FREE here (window rotation, no prologue
/// save/restore) — rv32 pays prologue stores for its 10 callee-saved pool
/// members; Xtensa pays only the amortized window-overflow traps (measured:
/// onset at call depth 6, +1 trap/frame steady state, P1 §3).
pub const ALLOC_POOL: &[PReg] = &[15, 14, 13, 12, 11, 10, 7, 6, 5, 4, 3, 2];

/// Pool members clobbered by a call (measured on silicon, P1 §2: caller
/// `a_j` survives a CALL8 iff `j < 8`). A call clobbers these; live vregs
/// must be saved/restored. Same order as their [`ALLOC_POOL`] prefix.
pub const CALLER_SAVED_POOL: &[PReg] = &[15, 14, 13, 12, 11, 10];

pub fn is_caller_saved_pool(r: PReg) -> bool {
    CALLER_SAVED_POOL.contains(&r)
}

/// Incoming (callee-view) argument register? Mirrors rv32's `is_arg_reg`
/// (which is also the incoming/precolor view there).
pub fn is_arg_reg(r: PReg) -> bool {
    (2..=7).contains(&r)
}

/// Outgoing (caller-view) argument staging register?
pub fn is_out_arg_reg(r: PReg) -> bool {
    (10..=15).contains(&r)
}

/// Call-preserved pool member (`a2..=a7` — survives CALL8 by rotation)?
/// The rv32 analogue is `is_callee_saved_pool_gpr` (s2–s11); here
/// "callee-saved" costs no prologue code.
#[inline]
pub fn is_callee_saved_pool(r: PReg) -> bool {
    (2..=7).contains(&r)
}

/// Parse register name to physical register number (standard Xtensa names:
/// `a0`–`a15`, plus the `sp` alias for `a1` accepted by gas).
#[allow(clippy::result_unit_err)] // rv32 shape parity: same signature as isa/rv32/gpr.rs
pub fn parse_reg(name: &str) -> Result<PReg, ()> {
    match name {
        "a0" => Ok(0),
        "a1" | "sp" => Ok(1),
        "a2" => Ok(2),
        "a3" => Ok(3),
        "a4" => Ok(4),
        "a5" => Ok(5),
        "a6" => Ok(6),
        "a7" => Ok(7),
        "a8" => Ok(8),
        "a9" => Ok(9),
        "a10" => Ok(10),
        "a11" => Ok(11),
        "a12" => Ok(12),
        "a13" => Ok(13),
        "a14" => Ok(14),
        "a15" => Ok(15),
        _ => Err(()),
    }
}

/// Name for debugging / text format. Xtensa has no per-role aliases the way
/// RISC-V does (`aN` is the canonical spelling; even `sp` disassembles as
/// `a1`), so this is the plain register name.
pub fn reg_name(reg: PReg) -> &'static str {
    match reg {
        0 => "a0",
        1 => "a1",
        2 => "a2",
        3 => "a3",
        4 => "a4",
        5 => "a5",
        6 => "a6",
        7 => "a7",
        8 => "a8",
        9 => "a9",
        10 => "a10",
        11 => "a11",
        12 => "a12",
        13 => "a13",
        14 => "a14",
        15 => "a15",
        _ => "???",
    }
}

#[inline]
pub fn pool_contains(r: PReg) -> bool {
    ALLOC_POOL.contains(&r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::emit::CallInc;

    #[test]
    fn test_parse_reg() {
        assert_eq!(parse_reg("a0"), Ok(0));
        assert_eq!(parse_reg("sp"), Ok(1));
        assert_eq!(parse_reg("a15"), Ok(15));
        assert_eq!(parse_reg("a16"), Err(()));
        assert_eq!(parse_reg("t0"), Err(()));
    }

    #[test]
    fn test_reg_name_roundtrip() {
        for i in 0..16u8 {
            let name = reg_name(i);
            assert_eq!(parse_reg(name), Ok(i), "Roundtrip failed for {i}");
        }
    }

    /// The caller-view constants are the callee-view constants shifted by the
    /// CALL8 window rotation, and that rotation matches the emitter's
    /// [`CallInc`] arithmetic.
    #[test]
    fn views_linked_by_call_rotation() {
        assert_eq!(CALL_ROTATION, 4 * CallInc::Call8.units());
        for i in 0..ARG_REGS.len() {
            assert_eq!(OUT_ARG_REGS[i], ARG_REGS[i] + CALL_ROTATION);
        }
        for i in 0..RET_REGS.len() {
            assert_eq!(CALL_RET_REGS[i], RET_REGS[i] + CALL_ROTATION);
        }
        assert_eq!(OUT_ARG_REGS[0], CallInc::Call8.arg_base());
    }

    /// Pool size and membership: 12 registers (vs rv32's 13), excluding
    /// exactly RA, SP, and the two emitter scratches.
    #[test]
    fn pool_size_and_exclusions() {
        assert_eq!(ALLOC_POOL.len(), 12);
        assert!(!pool_contains(RA_REG));
        assert!(!pool_contains(SP_REG));
        assert!(!pool_contains(SCRATCH));
        assert!(!pool_contains(SCRATCH2));
        // Every a-register is either pooled or one of the four reserved.
        for r in 0..16u8 {
            let reserved = r == RA_REG || r == SP_REG || r == SCRATCH || r == SCRATCH2;
            assert_eq!(pool_contains(r), !reserved, "a{r}");
        }
        // No duplicates.
        for (i, &a) in ALLOC_POOL.iter().enumerate() {
            assert!(!ALLOC_POOL[i + 1..].contains(&a), "duplicate a{a}");
        }
    }

    /// The caller-saved split matches the P1 silicon measurement: `a_j`
    /// survives a CALL8 iff `j < 8`.
    #[test]
    fn caller_saved_matches_measured_survival() {
        for &r in ALLOC_POOL {
            let survives_call8 = r < 8;
            assert_eq!(is_caller_saved_pool(r), !survives_call8, "a{r}");
            assert_eq!(is_callee_saved_pool(r), survives_call8, "a{r}");
        }
        // Front-loaded: the caller-saved bank is exactly the pool's prefix.
        assert_eq!(&ALLOC_POOL[..CALLER_SAVED_POOL.len()], CALLER_SAVED_POOL);
    }

    #[test]
    fn arg_predicates() {
        for &r in &ARG_REGS {
            assert!(is_arg_reg(r));
            assert!(!is_out_arg_reg(r));
        }
        for &r in &OUT_ARG_REGS {
            assert!(is_out_arg_reg(r));
            assert!(!is_arg_reg(r));
        }
        assert!(!is_arg_reg(SCRATCH));
        assert!(!is_out_arg_reg(SCRATCH2));
    }
}
