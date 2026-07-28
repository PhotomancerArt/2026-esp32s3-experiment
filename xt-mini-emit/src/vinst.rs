//! MiniVInst: a structural mirror of lpvm-native's `VInst` at the *emit stage*
//! (post register allocation).
//!
//! The real backend's pipeline is `LPIR -> lower -> VInst (virtual regs) ->
//! regalloc -> emit`. The emitter consumes VInsts whose operands have been
//! rewritten to physical registers. MiniVInst models exactly that input: the
//! same op categories and operand shapes as `lpvm-native/src/vinst.rs`, with
//! [`PReg`] (a physical Xtensa `a`-register number) where the real IR has
//! `VReg`. See the README mapping table for the field-by-field correspondence.
//!
//! Operand registers must be in `a2..=a7`: `a0`/`a1` are the windowed ABI's
//! return-address/stack-pointer, and `a8..=a15` are reserved for emitter
//! scratch and outgoing call arguments (all caller-clobbered across `callx8`).

extern crate alloc;

use alloc::vec::Vec;

/// Label id for branch targets (mirrors `lpvm_native::vinst::LabelId`).
pub type LabelId = u32;

/// A physical Xtensa address register `a{0..15}`, the output of register
/// allocation. MiniVInst programs may only use `a2..=a7` (see module docs).
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct PReg(pub u8);

impl PReg {
    pub const fn num(self) -> u8 {
        self.0
    }
}

impl core::fmt::Debug for PReg {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "a{}", self.0)
    }
}

/// Register-register ALU ops. Mirrors `lpvm_native::vinst::AluOp` (RISC-V
/// R-type names; the Xtensa lowering is the emitter's business).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AluOp {
    Add,
    Sub,
    Mul,
    /// High half of signed `src1 * src2`.
    MulH,
    And,
    Or,
    Xor,
    Sll,
    SrlU,
    SraS,
    DivS,
    DivU,
    RemS,
    RemU,
}

/// Register-immediate ALU ops. Mirrors `lpvm_native::vinst::AluImmOp`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AluImmOp {
    Addi,
    Andi,
    Ori,
    Xori,
    Slli,
    SrliU,
    SraiS,
    Slti,
    SltiU,
}

/// Comparison condition. Mirrors `lpvm_native::vinst::IcmpCond`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IcmpCond {
    Eq,
    Ne,
    LtS,
    LeS,
    GtS,
    GeS,
    LtU,
    LeU,
    GtU,
    GeU,
}

/// Call-target symbol id (mirrors `lpvm_native::vinst::SymbolId`, which
/// indexes `ModuleSymbols::names`; here it names an absolute-address literal
/// slot the host patches after emission).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SymbolId(pub u16);

/// A call target.
///
/// The real `VInst::Call` carries only a `SymbolId`; the emitter decides how
/// to reach it. Here the two reach-strategies are explicit:
///
/// - [`Callee::Func`]: another function in the same emitted buffer, reached
///   with a PC-relative `CALL8` — position-independent, so it dual-runs on
///   hardware where the load address is unknown.
/// - [`Callee::Sym`]: an absolute address held in a literal-pool slot,
///   reached with `L32R` + `CALLX8`. The slot's buffer offset is reported in
///   [`crate::emit::EmitOut::sym_slots`] for the host to patch — this is the
///   real-builtin path (the monorepo backend links builtin addresses the same
///   way), but it is position-dependent, so tests run it emulator-only (the
///   device loads payloads at heap-chosen addresses; hardware call coverage
///   comes from the position-independent [`Callee::Func`] case).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Callee {
    Func(usize),
    Sym(SymbolId),
}

/// The mini virtual instruction set: `lpvm_native::vinst::VInst` with
/// physical registers. `src_op` provenance fields are dropped (debug-only in
/// the real IR). See the README for the variant-by-variant mapping.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MiniVInst {
    /// `dst = src1 OP src2` — mirrors `VInst::AluRRR`.
    AluRRR {
        op: AluOp,
        dst: PReg,
        src1: PReg,
        src2: PReg,
    },
    /// `dst = src OP imm` — mirrors `VInst::AluRRI`.
    AluRRI {
        op: AluImmOp,
        dst: PReg,
        src: PReg,
        imm: i32,
    },
    /// `dst = (lhs COND rhs) ? 1 : 0` — mirrors `VInst::Icmp` (pseudo,
    /// multi-instruction expansion in the emitter).
    Icmp {
        dst: PReg,
        lhs: PReg,
        rhs: PReg,
        cond: IcmpCond,
    },
    /// `dst = (src COND imm) ? 1 : 0` — mirrors `VInst::IcmpImm`.
    IcmpImm {
        dst: PReg,
        src: PReg,
        imm: i32,
        cond: IcmpCond,
    },
    /// `dst = cond != 0 ? if_true : if_false` — mirrors `VInst::Select`.
    Select {
        dst: PReg,
        cond: PReg,
        if_true: PReg,
        if_false: PReg,
    },
    /// Unconditional branch — mirrors `VInst::Br`.
    Br { target: LabelId },
    /// Conditional branch: taken when `cond != 0` (or `cond == 0` when
    /// `invert`) — mirrors `VInst::BrIf`.
    BrIf {
        cond: PReg,
        target: LabelId,
        invert: bool,
    },
    /// Register copy — mirrors `VInst::Mov`.
    Mov { dst: PReg, src: PReg },
    /// Word load `dst = [base + offset]` — mirrors `VInst::Load32`.
    Load32 { dst: PReg, base: PReg, offset: i32 },
    /// Word store `[base + offset] = src` — mirrors `VInst::Store32`.
    Store32 { src: PReg, base: PReg, offset: i32 },
    /// Byte store (low 8 bits) — mirrors `VInst::Store8`.
    Store8 { src: PReg, base: PReg, offset: i32 },
    /// Halfword store (low 16 bits) — mirrors `VInst::Store16`.
    Store16 { src: PReg, base: PReg, offset: i32 },
    /// `dst = address of stack slot` — mirrors `VInst::SlotAddr`.
    SlotAddr { dst: PReg, slot: u32 },
    /// 32-bit constant load — mirrors `VInst::IConst32`.
    IConst32 { dst: PReg, val: i32 },
    /// Function call — mirrors `VInst::Call` (post-regalloc: `args`/`ret` are
    /// physical registers instead of `VRegSlice`s; the sret plumbing fields do
    /// not apply to the single-scalar mini ABI).
    Call {
        callee: Callee,
        args: Vec<PReg>,
        ret: Option<PReg>,
    },
    /// Return — mirrors `VInst::Ret` (single optional scalar instead of
    /// `VRegSlice`).
    Ret { val: Option<PReg> },
    /// Branch-target definition — mirrors `VInst::Label`.
    Label(LabelId),
    /// Fuel check — mirrors `VInst::FuelCheck`. `fuel_base` points at a
    /// 32-bit fuel counter in memory (the real IR's `vmctx`; the counter is
    /// the vmctx low fuel word). If the counter is observed at zero, branch
    /// to `trap_label`; otherwise, when `decrement`, subtract 1 and store
    /// back (check-then-decrement, matching the real semantics).
    FuelCheck {
        fuel_base: PReg,
        decrement: bool,
        trap_label: LabelId,
    },
}

/// A function: stack-slot declarations + instructions. Mirrors the shape the
/// real emit stage sees (slot sizes come from the LPIR frame; instructions
/// are the allocated VInst stream).
#[derive(Clone, Debug, Default)]
pub struct MiniFunc {
    /// Byte sizes of stack slots, indexed by `SlotAddr::slot`. Each is
    /// rounded up to 4 bytes when laid out.
    pub slots: Vec<u32>,
    pub insts: Vec<MiniVInst>,
}

/// A program: one or more functions emitted into a single buffer sharing one
/// literal pool. `funcs[0]` is the entry function (the runner/emulator calls
/// it with one `u32` argument arriving in `a2`; its return value is read from
/// `a2`).
#[derive(Clone, Debug, Default)]
pub struct MiniProgram {
    pub funcs: Vec<MiniFunc>,
}
