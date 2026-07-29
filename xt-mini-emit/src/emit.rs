//! The emitter: MiniVInst -> executable Xtensa bytes via `lp_xt_inst::encode`.
//!
//! Layout contract (hardware-proven in the spike, see repo FINDINGS.md):
//!
//! ```text
//! [ literal pool: 4-byte slots ][ func0: pad-to-4, ENTRY, code ][ func1 ... ]
//! ^ buffer offset 0                                  entry_offset = pool end
//! ```
//!
//! - **Literal pool before code**: `L32R` reaches literals *backward only*
//!   (`target = ((PC + 3) & !3) + (imm16 << 2)`, imm16 sign-extended), so a
//!   pool at the buffer start is reachable from anywhere in the first ~256 KB
//!   of code. Constants are deduplicated within the buffer (we own the whole
//!   buffer — the spike's warning was that *assembler* output dedups across
//!   an object and is therefore not self-contained).
//! - **Position independence**: branches, `J`, and `CALL8` are PC-relative;
//!   pooled *constants* are values, not addresses. The only position-dependent
//!   construct is a [`Callee::Sym`] literal (an absolute callee address),
//!   whose pool slot is reported in [`EmitOut::sym_slots`] for the host to
//!   patch once the load address is known.
//! - **Wide forms only** for lowered instructions (density is a later
//!   optimization); the 2-byte `nop.n` appears only in alignment padding.
//! - **Branch fixups**: label branches are layout items resolved by an
//!   iterative sizing pass. A conditional branch whose target is outside the
//!   `beqz`/`bnez` signed-12-bit range is relaxed to the inverted branch over
//!   an unconditional `J` (monotonic short->long, so the loop converges).
//!
//! Register model (windowed ABI, hardware-verified): the emitter consumes
//! the contract modules [`crate::gpr`] / [`crate::abi`] — every register
//! number below comes from there (the coherence proof that the contract
//! describes what actually runs on silicon).
//!
//! - We are a windowed callee: one `ENTRY a1, frame` prologue, `RETW`
//!   epilogue. Our argument arrives in `gpr::ARG_REGS[0]` (a2); our result
//!   is returned in `gpr::RET_REGS[0]` (a2).
//! - `gpr::RA_REG`/`gpr::SP_REG` are return-address/SP. Program registers
//!   are the preserved bank `gpr::ARG_REGS` (a2..=a7).
//! - `gpr::SCRATCH`/`gpr::SCRATCH2` (a8/a9) are emitter scratch.
//! - The **call increment** ([`CallInc`], default [`crate::abi::CALL_INC`] =
//!   `CALL8`) picks the rest: which program registers survive our own calls,
//!   where outgoing arguments are staged (callee `a2..` = caller
//!   `a[4*inc]+2..`), and where a call's result comes back (caller
//!   `a[4*inc + 2]`). Under `CALL8` that is the M5 layout: `a2..=a7` all
//!   survive, args stage at `gpr::OUT_ARG_REGS` (a10..=a15), result in
//!   `gpr::CALL_RET_REGS[0]` (a10). See `docs/call-inc-study.md` for the
//!   measured tradeoff.
//! - Frame: `ENTRY` needs a 16-byte base save area at the frame top, plus
//!   16 bytes per additional window unit for the `a4..` spills written by
//!   `_WindowOverflow{8,12}` — the top [`CallInc::save_area_bytes`] of the
//!   frame (= [`crate::abi::FRAME_TOP_RESERVED_BYTES`] under the default
//!   policy) are reserved and stack slots grow from `a1 + 0` upward.

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;

use lp_xt_inst::{
    encode, AluRrr, AluRs, AluRt, BrRr, BrZ, CallOp, CallxOp, Inst, LoadOp, NullaryNarrowOp,
    NullaryOp, Reg, ShiftSetOp, StoreOp,
};

use crate::gpr;
use crate::vinst::{
    AluImmOp, AluOp, Callee, IcmpCond, LabelId, MiniFunc, MiniProgram, MiniVInst, PReg, SymbolId,
};

/// Emitter scratch registers (never available to MiniVInst programs) —
/// [`gpr::SCRATCH`]/[`gpr::SCRATCH2`] as encoder operands.
const SCRATCH0: Reg = Reg::new(gpr::SCRATCH);
const SCRATCH1: Reg = Reg::new(gpr::SCRATCH2);

/// The stack pointer as an encoder operand ([`gpr::SP_REG`]).
const SP: Reg = Reg::new(gpr::SP_REG);

/// The windowed call increment the emitter uses for every call it produces.
///
/// `CALLn` stages the return address in the caller's `a[n]` and the callee's
/// `ENTRY` rotates the window so callee `a0` = caller `a[n]`. The choice
/// fixes, for the caller:
///
/// - **preserved registers**: caller `a2..a[n-1]` survive the call (they sit
///   below the callee's window);
/// - **argument staging**: callee arguments always arrive in its `a2..=a7`,
///   i.e. caller `a[n+2]..a[n+7]` — but the caller can only write inside its
///   own 16-register window, so `CALL12` can stage just 2 register args
///   (`a14`,`a15`);
/// - **overflow pressure**: each live frame owns `n/4` base-units of the
///   64-register file, so smaller increments fit more frames before the
///   window-overflow traps start spilling.
///
/// Measured tradeoff (emulator + ESP32-S3): `docs/call-inc-study.md`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CallInc {
    /// `CALL4`: 2 preserved program registers, 6 register args, ~cheapest
    /// window pressure.
    Call4,
    /// `CALL8`: 6 preserved program registers, 6 register args. The policy
    /// choice ([`crate::abi::CALL_INC`], the hardware-proven M5 layout).
    Call8,
    /// `CALL12`: 10 preserved registers, but only **2** register args.
    Call12,
}

impl Default for CallInc {
    /// The ABI policy constant — `abi.rs` is the single source of truth.
    fn default() -> Self {
        crate::abi::CALL_INC
    }
}

impl CallInc {
    /// Window rotation in 4-register base units (the value of `PS.CALLINC`).
    pub const fn units(self) -> u8 {
        match self {
            CallInc::Call4 => 1,
            CallInc::Call8 => 2,
            CallInc::Call12 => 3,
        }
    }

    /// Caller register that maps to the callee's first argument register
    /// [`gpr::ARG_REGS`]`[0]` (first argument out, and where the callee's
    /// return value lands). Under the default policy this is
    /// [`gpr::OUT_ARG_REGS`]`[0]` (asserted in gpr's tests).
    pub const fn arg_base(self) -> u8 {
        4 * self.units() + gpr::ARG_REGS[0]
    }

    /// Register-argument capacity: the callee reads args from its
    /// [`gpr::ARG_REGS`] (`a2..=a7`), but the caller can only stage into its
    /// own window (`a15` max), so capacity =
    /// `min(ARG_REGS.len(), 16 - arg_base)` — 6 for CALL4/CALL8, **2** for
    /// CALL12.
    pub const fn max_reg_args(self) -> usize {
        let cap = (16 - self.arg_base()) as usize;
        if cap > gpr::ARG_REGS.len() {
            gpr::ARG_REGS.len()
        } else {
            cap
        }
    }

    /// Reserved bytes at the top of every frame for window-trap spills.
    ///
    /// A frame entered with increment `u` units needs the 16-byte base save
    /// area plus `16 * (u - 1)` bytes for the `a4..` registers spilled by
    /// `_WindowOverflow{8,12}` — i.e. `16 * u`. The entry function is always
    /// reached via the runner's CALL8 (`u = 2`) regardless of the internal
    /// policy, so the reservation is floored at 32 bytes; `CALL12` raises it
    /// to 48. (Hardware-minimal would be per-function — 16 bytes for a
    /// CALL4-only callee — but the uniform reservation is safe under both
    /// the hardware handlers and lp-xt-emu's save-area model, and frame size
    /// does not affect window-overflow onset.)
    pub const fn save_area_bytes(self) -> u32 {
        // Floor at CALL8's units: the runner enters payloads via CALL8.
        let floor = CallInc::Call8.units();
        let u = self.units();
        let u = if u < floor { floor } else { u };
        16 * u as u32
    }

    /// PC-relative call opcode for this increment.
    pub const fn call_op(self) -> CallOp {
        match self {
            CallInc::Call4 => CallOp::Call4,
            CallInc::Call8 => CallOp::Call8,
            CallInc::Call12 => CallOp::Call12,
        }
    }

    /// Register-indirect call opcode for this increment.
    pub const fn callx_op(self) -> CallxOp {
        match self {
            CallInc::Call4 => CallxOp::Callx4,
            CallInc::Call8 => CallxOp::Callx8,
            CallInc::Call12 => CallxOp::Callx12,
        }
    }
}

/// `beqz`/`bnez` (BRI12) taken-target range: `PC + 4 + imm12`, imm12 signed.
const BRI12_MIN: i64 = -2048;
const BRI12_MAX: i64 = 2047;
/// `J` (CALL-format, 18-bit signed byte offset).
const J_MIN: i64 = -(1 << 17);
const J_MAX: i64 = (1 << 17) - 1;

/// Result of emitting a [`MiniProgram`].
#[derive(Clone, Debug)]
pub struct EmitOut {
    /// The buffer: literal pool followed by code. Must be loaded at a
    /// 4-byte-aligned address (both the emulator and the device runner do).
    pub code: Vec<u8>,
    /// Entry offset of `funcs[0]` (== the pool size; always 4-aligned).
    pub entry_offset: u32,
    /// Entry offset of every function, by index.
    pub func_offsets: Vec<u32>,
    /// Buffer offsets of the literal-pool slots backing [`Callee::Sym`]
    /// calls. The host must overwrite each slot with the absolute address of
    /// the symbol before executing the buffer.
    pub sym_slots: Vec<(SymbolId, u32)>,
}

/// Emit `prog` into a single self-contained buffer with the default
/// [`CallInc::Call8`] policy.
///
/// Panics on malformed input (registers outside `a2..=a7`, unknown labels,
/// more call args than the increment supports, immediate fields out of their
/// documented ranges) — these are compiler-invariant violations, mirroring
/// the real emit stage where they are unreachable by construction.
pub fn emit_program(prog: &MiniProgram) -> EmitOut {
    emit_program_with(prog, CallInc::default())
}

/// As [`emit_program`], with an explicit call-increment policy. The same
/// MiniVInst program can be emitted under CALL4/CALL8/CALL12 for comparison;
/// a program whose calls carry more than [`CallInc::max_reg_args`] arguments
/// cannot be emitted under that increment and panics.
pub fn emit_program_with(prog: &MiniProgram, inc: CallInc) -> EmitOut {
    assert!(!prog.funcs.is_empty(), "program has no functions");
    let mut e = Emitter {
        inc,
        ..Emitter::default()
    };
    for (i, f) in prog.funcs.iter().enumerate() {
        e.lower_func(i, f, prog.funcs.len());
    }
    e.finish()
}

// ---------------------------------------------------------------------------
// Literals
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Literal {
    /// A known 32-bit constant (deduplicated by value).
    Const(u32),
    /// An absolute symbol address, patched by the host (deduplicated by id).
    Sym(SymbolId),
}

// ---------------------------------------------------------------------------
// Layout items
// ---------------------------------------------------------------------------

/// Label key: (function index, label id). Each function has its own label
/// namespace, mirroring the per-function `LabelId`s of the real IR.
type GLabel = (usize, LabelId);

#[derive(Clone, Debug)]
enum Item {
    /// Fixed, already-encoded bytes.
    Bytes(Vec<u8>),
    /// `beqz`/`bnez reg, label` — may relax to the inverted form over a `J`.
    CondBr { nez: bool, reg: Reg, label: GLabel },
    /// `j label`.
    Jump { label: GLabel },
    /// `l32r rt, <literal>`.
    L32r { rt: Reg, lit: usize },
    /// `call{4,8,12} <function entry>` (PC-relative, target 4-aligned; the
    /// opcode comes from the emitter's [`CallInc`]).
    CallFunc { func: usize },
    /// Function start: pad to 4-byte alignment (executable nops), record the
    /// entry offset.
    FuncStart(usize),
    /// Label definition (zero size).
    LabelDef(GLabel),
}

#[derive(Clone, Debug)]
struct Slot {
    item: Item,
    /// Byte offset within the buffer (valid after layout).
    offset: u32,
    /// CondBr only: relaxed to the long (branch-over-`J`) form.
    long: bool,
}

// ---------------------------------------------------------------------------
// Emitter
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Emitter {
    items: Vec<Slot>,
    literals: Vec<Literal>,
    n_funcs: usize,
    /// Call-increment policy for every call this emitter produces.
    inc: CallInc,
}

impl Emitter {
    // --- item/byte helpers -------------------------------------------------

    fn push(&mut self, item: Item) {
        self.items.push(Slot {
            item,
            offset: 0,
            long: false,
        });
    }

    /// Append an encoded instruction to the current `Bytes` run.
    fn inst(&mut self, i: Inst) {
        let bytes = encode(&i);
        if let Some(Slot {
            item: Item::Bytes(v),
            ..
        }) = self.items.last_mut()
        {
            v.extend_from_slice(&bytes);
        } else {
            self.push(Item::Bytes(bytes));
        }
    }

    fn lit(&mut self, l: Literal) -> usize {
        if let Some(i) = self.literals.iter().position(|&x| x == l) {
            return i;
        }
        self.literals.push(l);
        self.literals.len() - 1
    }

    /// `mov rd, rs` (wide form: `or rd, rs, rs`, as the assembler emits).
    fn mov(&mut self, rd: Reg, rs: Reg) {
        if rd != rs {
            self.inst(Inst::Rrr(AluRrr::Or, rd, rs, rs));
        }
    }

    /// Materialize a 32-bit constant into `rd` (`movi` or pooled `l32r`).
    fn iconst(&mut self, rd: Reg, val: i32) {
        if (-2048..=2047).contains(&val) {
            self.inst(Inst::Movi(rd, val));
        } else {
            let lit = self.lit(Literal::Const(val as u32));
            self.push(Item::L32r { rt: rd, lit });
        }
    }

    /// `rd = rs + imm`, using `addi`/`addmi` when they fit.
    fn add_imm(&mut self, rd: Reg, rs: Reg, imm: i32) {
        if imm == 0 {
            self.mov(rd, rs);
        } else if (-128..=127).contains(&imm) {
            self.inst(Inst::Addi(rd, rs, imm));
        } else if (-32768..=32512).contains(&imm) && imm % 256 == 0 {
            self.inst(Inst::Addmi(rd, rs, imm));
        } else if (-32768..=32639).contains(&imm) {
            // addmi (high byte) + addi (low byte, signed).
            let low = (imm << 24) >> 24; // sign-extended low 8 bits
            let high = imm - low; // multiple of 256
            self.inst(Inst::Addmi(rd, rs, high));
            self.inst(Inst::Addi(rd, rd, low));
        } else {
            self.iconst(SCRATCH1, imm);
            self.inst(Inst::Rrr(AluRrr::Add, rd, rs, SCRATCH1));
        }
    }

    // --- register checking -------------------------------------------------

    /// Convert a program register, enforcing the preserved-bank contract:
    /// MiniVInst programs use only [`gpr::ALLOC_POOL`]'s call-preserved bank
    /// (`a2..=a7`) — this mini emitter uses the caller-saved bank for call
    /// staging instead of allocating it (the full backend pools it too and
    /// resolves staging conflicts in regalloc).
    fn r(&self, p: PReg) -> Reg {
        assert!(
            gpr::is_callee_saved_pool(p.num()),
            "MiniVInst register {p:?} outside the program range a2..=a7"
        );
        Reg::new(p.num())
    }

    // --- lowering ----------------------------------------------------------

    fn lower_func(&mut self, idx: usize, f: &MiniFunc, n_funcs: usize) {
        self.n_funcs = n_funcs;
        self.push(Item::FuncStart(idx));

        // Frame: reserved save areas at the top, slots from a1+0 upward.
        let slot_offsets: Vec<u32> = {
            let mut offs = Vec::with_capacity(f.slots.len());
            let mut at = 0u32;
            for &sz in &f.slots {
                offs.push(at);
                at += sz.div_ceil(4) * 4;
            }
            offs
        };
        let slots_bytes = slot_offsets
            .last()
            .zip(f.slots.last())
            .map_or(0, |(&o, &s)| o + s.div_ceil(4) * 4);
        let align = crate::abi::STACK_ALIGNMENT;
        let frame = (self.inc.save_area_bytes() + slots_bytes).div_ceil(align) * align;
        assert!(frame <= 32760, "frame too large for ENTRY immediate");
        self.inst(Inst::Entry(SP, frame));

        for inst in &f.insts {
            self.lower_inst(idx, inst, &slot_offsets);
        }
    }

    fn lower_inst(&mut self, fidx: usize, mi: &MiniVInst, slot_offsets: &[u32]) {
        match mi {
            MiniVInst::AluRRR {
                op,
                dst,
                src1,
                src2,
            } => {
                let (d, s1, s2) = (self.r(*dst), self.r(*src1), self.r(*src2));
                self.alu_rrr(*op, d, s1, s2);
            }
            MiniVInst::AluRRI { op, dst, src, imm } => {
                let (d, s) = (self.r(*dst), self.r(*src));
                self.alu_rri(*op, d, s, *imm);
            }
            MiniVInst::Icmp {
                dst,
                lhs,
                rhs,
                cond,
            } => {
                let (d, l, r) = (self.r(*dst), self.r(*lhs), self.r(*rhs));
                self.icmp(d, l, r, *cond);
            }
            MiniVInst::IcmpImm {
                dst,
                src,
                imm,
                cond,
            } => {
                let (d, s) = (self.r(*dst), self.r(*src));
                self.iconst(SCRATCH1, *imm);
                self.icmp(d, s, SCRATCH1, *cond);
            }
            MiniVInst::Select {
                dst,
                cond,
                if_true,
                if_false,
            } => {
                let (d, c, t, f) = (
                    self.r(*dst),
                    self.r(*cond),
                    self.r(*if_true),
                    self.r(*if_false),
                );
                // Compute in scratch so dst may alias any input.
                self.mov(SCRATCH0, f);
                self.inst(Inst::Rrr(AluRrr::Movnez, SCRATCH0, t, c));
                self.mov(d, SCRATCH0);
            }
            MiniVInst::Br { target } => self.push(Item::Jump {
                label: (fidx, *target),
            }),
            MiniVInst::BrIf {
                cond,
                target,
                invert,
            } => {
                let c = self.r(*cond);
                self.push(Item::CondBr {
                    nez: !invert,
                    reg: c,
                    label: (fidx, *target),
                });
            }
            MiniVInst::Mov { dst, src } => {
                let (d, s) = (self.r(*dst), self.r(*src));
                self.mov(d, s);
            }
            MiniVInst::Load32 { dst, base, offset } => {
                let (d, b) = (self.r(*dst), self.r(*base));
                let (b, off) = self.mem_addr(b, *offset, 1020, 4);
                self.inst(Inst::Load(LoadOp::L32i, d, b, off));
            }
            MiniVInst::Store32 { src, base, offset } => {
                let (s, b) = (self.r(*src), self.r(*base));
                let (b, off) = self.mem_addr(b, *offset, 1020, 4);
                self.inst(Inst::Store(StoreOp::S32i, s, b, off));
            }
            MiniVInst::Store8 { src, base, offset } => {
                let (s, b) = (self.r(*src), self.r(*base));
                let (b, off) = self.mem_addr(b, *offset, 255, 1);
                self.inst(Inst::Store(StoreOp::S8i, s, b, off));
            }
            MiniVInst::Store16 { src, base, offset } => {
                let (s, b) = (self.r(*src), self.r(*base));
                let (b, off) = self.mem_addr(b, *offset, 510, 2);
                self.inst(Inst::Store(StoreOp::S16i, s, b, off));
            }
            MiniVInst::SlotAddr { dst, slot } => {
                let d = self.r(*dst);
                let off = *slot_offsets
                    .get(*slot as usize)
                    .unwrap_or_else(|| panic!("SlotAddr: slot {slot} not declared"));
                self.add_imm(d, SP, off as i32);
            }
            MiniVInst::IConst32 { dst, val } => {
                let d = self.r(*dst);
                self.iconst(d, *val);
            }
            MiniVInst::Call { callee, args, ret } => {
                let inc = self.inc;
                assert!(
                    args.len() <= inc.max_reg_args(),
                    "at most {} register args under {inc:?} (got {})",
                    inc.max_reg_args(),
                    args.len()
                );
                // Stage args at the increment's staging area (under the
                // default CALL8 policy: gpr::OUT_ARG_REGS). Under CALL8/12
                // the staging registers (a10+/a14+) never alias the a2..=a7
                // sources; under CALL4 the area is a6..a11, so dests inside
                // the program bank (a6/a7) would clobber still-unread
                // sources — bounce those through the registers just above
                // the 6-slot staging area (a12/a13 for CALL4: free — above
                // staging, below nothing live) and write them last.
                let base = inc.arg_base();
                let mut bounced: [Option<(u8, u8)>; 2] = [None; 2];
                for (i, a) in args.iter().enumerate() {
                    let s = self.r(*a);
                    let dest = base + i as u8;
                    if gpr::is_callee_saved_pool(dest) {
                        let tmp = base + gpr::ARG_REGS.len() as u8 + i as u8;
                        self.mov(Reg::new(tmp), s);
                        bounced[i] = Some((dest, tmp));
                    } else {
                        self.mov(Reg::new(dest), s);
                    }
                }
                for (dest, tmp) in bounced.into_iter().flatten() {
                    self.mov(Reg::new(dest), Reg::new(tmp));
                }
                match callee {
                    Callee::Func(f) => {
                        assert!(*f < self.n_funcs, "Call to unknown function {f}");
                        self.push(Item::CallFunc { func: *f });
                    }
                    Callee::Sym(sym) => {
                        let lit = self.lit(Literal::Sym(*sym));
                        self.push(Item::L32r { rt: SCRATCH0, lit });
                        self.inst(Inst::Callx(inc.callx_op(), SCRATCH0));
                    }
                }
                if let Some(rr) = ret {
                    // The callee's RET_REGS[0] rotates back to caller
                    // `arg_base` (= gpr::CALL_RET_REGS[0] under CALL8).
                    let d = self.r(*rr);
                    self.mov(d, Reg::new(base));
                }
            }
            MiniVInst::Ret { val } => {
                if let Some(v) = val {
                    let s = self.r(*v);
                    self.mov(Reg::new(gpr::RET_REGS[0]), s);
                }
                self.inst(Inst::Nullary(NullaryOp::Retw));
            }
            MiniVInst::Label(l) => self.push(Item::LabelDef((fidx, *l))),
            MiniVInst::FuelCheck {
                fuel_base,
                decrement,
                trap_label,
            } => {
                let b = self.r(*fuel_base);
                self.inst(Inst::Load(LoadOp::L32i, SCRATCH0, b, 0));
                self.push(Item::CondBr {
                    nez: false,
                    reg: SCRATCH0,
                    label: (fidx, *trap_label),
                });
                if *decrement {
                    self.inst(Inst::Addi(SCRATCH0, SCRATCH0, -1));
                    self.inst(Inst::Store(StoreOp::S32i, SCRATCH0, b, 0));
                }
            }
        }
    }

    /// Reduce a load/store address to an encodable (base, offset) pair, going
    /// through scratch when the offset is negative or out of range.
    fn mem_addr(&mut self, base: Reg, offset: i32, max: i32, align: i32) -> (Reg, u32) {
        if (0..=max).contains(&offset) && offset % align == 0 {
            (base, offset as u32)
        } else {
            self.add_imm(SCRATCH0, base, offset);
            (SCRATCH0, 0)
        }
    }

    fn alu_rrr(&mut self, op: AluOp, d: Reg, s1: Reg, s2: Reg) {
        let direct = match op {
            AluOp::Add => Some(AluRrr::Add),
            AluOp::Sub => Some(AluRrr::Sub),
            AluOp::Mul => Some(AluRrr::Mull),
            AluOp::MulH => Some(AluRrr::Mulsh),
            AluOp::And => Some(AluRrr::And),
            AluOp::Or => Some(AluRrr::Or),
            AluOp::Xor => Some(AluRrr::Xor),
            AluOp::DivS => Some(AluRrr::Quos),
            AluOp::DivU => Some(AluRrr::Quou),
            AluOp::RemS => Some(AluRrr::Rems),
            AluOp::RemU => Some(AluRrr::Remu),
            AluOp::Sll | AluOp::SrlU | AluOp::SraS => None,
        };
        if let Some(x) = direct {
            self.inst(Inst::Rrr(x, d, s1, s2));
            return;
        }
        // Register-amount shifts go through SAR. Xtensa shifts use amounts
        // mod 32 via SSL/SSR, matching the RISC-V `& 31` semantics.
        match op {
            AluOp::Sll => {
                self.inst(Inst::ShiftSet(ShiftSetOp::Ssl, s2));
                self.inst(Inst::Rs(AluRs::Sll, d, s1));
            }
            AluOp::SrlU => {
                self.inst(Inst::ShiftSet(ShiftSetOp::Ssr, s2));
                self.inst(Inst::Rt(AluRt::Srl, d, s1));
            }
            AluOp::SraS => {
                self.inst(Inst::ShiftSet(ShiftSetOp::Ssr, s2));
                self.inst(Inst::Rt(AluRt::Sra, d, s1));
            }
            _ => unreachable!(),
        }
    }

    fn alu_rri(&mut self, op: AluImmOp, d: Reg, s: Reg, imm: i32) {
        match op {
            AluImmOp::Addi => self.add_imm(d, s, imm),
            AluImmOp::Andi | AluImmOp::Ori | AluImmOp::Xori => {
                // Xtensa has no and/or/xor-immediate: materialize + RRR.
                self.iconst(SCRATCH1, imm);
                let x = match op {
                    AluImmOp::Andi => AluRrr::And,
                    AluImmOp::Ori => AluRrr::Or,
                    _ => AluRrr::Xor,
                };
                self.inst(Inst::Rrr(x, d, s, SCRATCH1));
            }
            AluImmOp::Slli => {
                let sa = imm as u32 & 31;
                if sa == 0 {
                    self.mov(d, s);
                } else {
                    self.inst(Inst::Slli(d, s, sa as u8));
                }
            }
            AluImmOp::SrliU => {
                let sa = imm as u32 & 31;
                if sa == 0 {
                    self.mov(d, s);
                } else if sa <= 15 {
                    self.inst(Inst::Srli(d, s, sa as u8));
                } else {
                    // srli >15 has no encoding; extract the top (32-sa) bits.
                    self.inst(Inst::Extui(d, s, sa as u8, (32 - sa) as u8));
                }
            }
            AluImmOp::SraiS => {
                let sa = imm as u32 & 31;
                if sa == 0 {
                    self.mov(d, s);
                } else {
                    self.inst(Inst::Srai(d, s, sa as u8));
                }
            }
            AluImmOp::Slti => {
                self.iconst(SCRATCH1, imm);
                self.icmp(d, s, SCRATCH1, IcmpCond::LtS);
            }
            AluImmOp::SltiU => {
                self.iconst(SCRATCH1, imm);
                self.icmp(d, s, SCRATCH1, IcmpCond::LtU);
            }
        }
    }

    /// `d = (l COND r) ? 1 : 0`. `r` may be SCRATCH1; computes in SCRATCH0 so
    /// `d` may alias `l`/`r`.
    fn icmp(&mut self, d: Reg, l: Reg, r: Reg, cond: IcmpCond) {
        // Map to a branch-if-true `op rs, rt` (swapping operands for the
        // conditions Xtensa lacks).
        let (op, rs, rt) = match cond {
            IcmpCond::Eq => (BrRr::Beq, l, r),
            IcmpCond::Ne => (BrRr::Bne, l, r),
            IcmpCond::LtS => (BrRr::Blt, l, r),
            IcmpCond::GeS => (BrRr::Bge, l, r),
            IcmpCond::LtU => (BrRr::Bltu, l, r),
            IcmpCond::GeU => (BrRr::Bgeu, l, r),
            IcmpCond::GtS => (BrRr::Blt, r, l),
            IcmpCond::LeS => (BrRr::Bge, r, l),
            IcmpCond::GtU => (BrRr::Bltu, r, l),
            IcmpCond::LeU => (BrRr::Bgeu, r, l),
        };
        self.inst(Inst::Movi(SCRATCH0, 1));
        // Branch over the following 3-byte movi: target = PC + 4 + 2.
        self.inst(Inst::BranchRr(op, rs, rt, 2));
        self.inst(Inst::Movi(SCRATCH0, 0));
        self.mov(d, SCRATCH0);
    }

    // --- layout + final encode --------------------------------------------

    fn finish(mut self) -> EmitOut {
        let pool_bytes = 4 * self.literals.len() as u32;

        // Iterative sizing: item sizes depend on offsets (alignment padding)
        // and on short/long conditional-branch choice. Relaxation is
        // monotonic (short -> long only), so this converges.
        for iteration in 0.. {
            assert!(iteration < 64, "branch relaxation failed to converge");
            let mut off = pool_bytes;
            for s in &mut self.items {
                s.offset = off;
                off += item_size(&s.item, off, s.long);
            }
            let labels = self.label_offsets();
            let mut changed = false;
            for i in 0..self.items.len() {
                let s = &self.items[i];
                if s.long {
                    continue;
                }
                if let Item::CondBr { label, .. } = s.item {
                    let target = labels.resolve(label);
                    let diff = target as i64 - (s.offset as i64 + 4);
                    if !(BRI12_MIN..=BRI12_MAX).contains(&diff) {
                        self.items[i].long = true;
                        changed = true;
                    }
                }
            }
            if !changed {
                break;
            }
        }

        // Final encode.
        let labels = self.label_offsets();
        let total = self.items.last().map_or(pool_bytes, |s| {
            s.offset + item_size(&s.item, s.offset, s.long)
        });
        let mut code = vec![0u8; pool_bytes as usize];
        let mut sym_slots = Vec::new();
        for (i, l) in self.literals.iter().enumerate() {
            match l {
                Literal::Const(v) => {
                    code[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
                }
                Literal::Sym(s) => sym_slots.push((*s, 4 * i as u32)),
            }
        }

        let mut func_offsets = vec![0u32; labels.funcs.len()];
        for (i, &o) in labels.funcs.iter().enumerate() {
            func_offsets[i] = o;
        }

        for s in &self.items {
            let pc = s.offset as i64;
            match &s.item {
                Item::Bytes(b) => code.extend_from_slice(b),
                Item::LabelDef(_) => {}
                Item::FuncStart(_) => {
                    // Executable nop padding to 4-byte alignment.
                    let gap = (4 - (s.offset % 4)) % 4;
                    match gap {
                        0 => {}
                        2 => {
                            code.extend_from_slice(&encode(&Inst::NullaryN(NullaryNarrowOp::NopN)))
                        }
                        3 => code.extend_from_slice(&encode(&Inst::Nullary(NullaryOp::Nop))),
                        1 => {
                            code.extend_from_slice(&encode(&Inst::Nullary(NullaryOp::Nop)));
                            code.extend_from_slice(&encode(&Inst::NullaryN(NullaryNarrowOp::NopN)));
                        }
                        _ => unreachable!(),
                    }
                }
                Item::Jump { label } => {
                    let target = labels.resolve(*label) as i64;
                    code.extend_from_slice(&encode(&Inst::J(j_off(pc, target))));
                }
                Item::CondBr { nez, reg, label } => {
                    let target = labels.resolve(*label) as i64;
                    let kind = |nez| if nez { BrZ::Bnez } else { BrZ::Beqz };
                    if !s.long {
                        let diff = target - (pc + 4);
                        debug_assert!((BRI12_MIN..=BRI12_MAX).contains(&diff));
                        code.extend_from_slice(&encode(&Inst::BranchZ(
                            kind(*nez),
                            *reg,
                            diff as i32,
                        )));
                    } else {
                        // Inverted branch over `j target`: the branch skips
                        // the 3-byte J (target = PC + 4 + 2).
                        code.extend_from_slice(&encode(&Inst::BranchZ(kind(!*nez), *reg, 2)));
                        code.extend_from_slice(&encode(&Inst::J(j_off(pc + 3, target))));
                    }
                }
                Item::L32r { rt, lit } => {
                    let lit_off = 4 * *lit as i64;
                    // target = ((PC + 3) & !3) + (imm16 << 2); pool-at-start
                    // makes every offset backward (imm16 < 0). Valid for any
                    // 4-aligned load address.
                    let base = (pc + 3) & !3;
                    let imm = (lit_off - base) >> 2;
                    assert!(
                        (-32768..0).contains(&imm),
                        "L32R offset out of backward range (imm16 = {imm})"
                    );
                    code.extend_from_slice(&encode(&Inst::L32r(*rt, imm as i16 as u16)));
                }
                Item::CallFunc { func } => {
                    let target = labels.funcs[*func] as i64;
                    // target = (PC & !3) + (off << 2) + 4; both target and the
                    // rounded PC are 4-aligned, so the division is exact.
                    let off = (target - (pc & !3) - 4) >> 2;
                    assert!((J_MIN..=J_MAX).contains(&off), "CALLn offset out of range");
                    code.extend_from_slice(&encode(&Inst::Call(self.inc.call_op(), off as i32)));
                }
            }
        }
        debug_assert_eq!(code.len() as u32, total);

        EmitOut {
            entry_offset: func_offsets[0],
            code,
            func_offsets,
            sym_slots,
        }
    }

    fn label_offsets(&self) -> Labels {
        let mut labels = Labels {
            map: Vec::new(),
            funcs: vec![u32::MAX; self.n_funcs],
        };
        for s in &self.items {
            match s.item {
                Item::LabelDef(l) => labels.map.push((l, s.offset)),
                Item::FuncStart(f) => {
                    // Entry is the offset *after* alignment padding.
                    labels.funcs[f] = s.offset + align_pad(s.offset);
                }
                _ => {}
            }
        }
        labels
    }
}

/// `J` offset for a jump at `pc` targeting `target` (target = PC + 4 + off).
fn j_off(pc: i64, target: i64) -> i32 {
    let off = target - (pc + 4);
    assert!((J_MIN..=J_MAX).contains(&off), "J offset out of range");
    off as i32
}

fn align_pad(off: u32) -> u32 {
    let gap = (4 - (off % 4)) % 4;
    if gap == 1 {
        5 // nop (3) + nop.n (2)
    } else {
        gap
    }
}

fn item_size(item: &Item, off: u32, long: bool) -> u32 {
    match item {
        Item::Bytes(b) => b.len() as u32,
        Item::LabelDef(_) => 0,
        Item::FuncStart(_) => align_pad(off),
        Item::Jump { .. } | Item::L32r { .. } | Item::CallFunc { .. } => 3,
        Item::CondBr { .. } => {
            if long {
                6
            } else {
                3
            }
        }
    }
}

struct Labels {
    map: Vec<(GLabel, u32)>,
    funcs: Vec<u32>,
}

impl Labels {
    fn resolve(&self, l: GLabel) -> u32 {
        self.map
            .iter()
            .find(|(k, _)| *k == l)
            .map(|(_, o)| *o)
            .unwrap_or_else(|| panic!("undefined label {l:?}"))
    }
}
