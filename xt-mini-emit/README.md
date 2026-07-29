# xt-mini-emit

Prototype **MiniVInst → Xtensa (ESP32-S3 / LX7) code emitter** — milestone M5 of the
standalone-core roadmap. This crate solves every hard emitter sub-problem (literal
pools, branch fixups/relaxation, `ENTRY` frames, the windowed call ABI) on a small
structural mirror of lightplayer's real backend IR, so the monorepo port into
`lpvm-native/src/isa/xt/emit.rs` is mechanical substitution rather than fresh design.

All machine encodings go through `lp-xt-inst::encode` — this crate never writes
instruction bytes by hand. Every emitted program is validated by **dual-run**:
`lp-xt-emu` always, and the real ESP32-S3 (via `xt-runner-client`) when a device is
attached; the two must agree.

```bash
cargo test -p xt-mini-emit                                              # emit + emulator
XT_DEVICE_PORT=/dev/cu.usbmodem1101 cargo test -p xt-mini-emit -- --test-threads=1
```

## MiniVInst ↔ VInst mapping

`MiniVInst` mirrors `lpvm-native/src/vinst.rs`'s `VInst` **at the emit stage**: the
real pipeline is `LPIR → lower → VInst(VReg) → regalloc → emit`, and the emitter sees
VInsts whose operands are physical registers. MiniVInst models that input directly —
`PReg` (a physical `a`-register number, the regalloc output) where the IR has `VReg`.
`src_op` provenance fields (debug-only) are dropped throughout.

| `lpvm_native::vinst::VInst` | `MiniVInst` | Notes for the backporter |
|---|---|---|
| `AluRRR { op: AluOp, dst, src1, src2 }` | same shape | Same `AluOp` roster (Add..RemU). Xtensa lowering: direct RRR for add/sub/mul/mulh/logic/div/rem (`mull`, `mulsh`, `quos/quou`, `rems/remu`); shifts go through SAR (`ssl`/`ssr` + `sll`/`srl`/`sra`). |
| `AluRRI { op: AluImmOp, dst, src, imm }` | same shape | Same `AluImmOp` roster. Xtensa has no and/or/xor-immediate → materialize + RRR. `Addi` → `addi`/`addmi`/pair/pool. `SrliU` >15 → `extui`. `Slti/SltiU` → icmp expansion. |
| `Icmp { dst, lhs, rhs, cond }` | same shape | Pseudo-expansion: `movi scratch,1; b<cond> …,+skip; movi scratch,0; mov dst` (branch table below). Conditions Xtensa lacks (`Gt*`, `Le*`) swap operands. |
| `IcmpImm { dst, src, imm, cond }` | same shape | Materialize imm into scratch, then the `Icmp` expansion. |
| `Select { dst, cond, if_true, if_false }` | same shape | `mov scratch,if_false; movnez scratch,if_true,cond; mov dst,scratch` (aliasing-safe). |
| `Br { target: LabelId }` | same shape | `j` (18-bit signed, PC-relative). |
| `BrIf { cond, target, invert }` | same shape | `bnez`/`beqz` (BRI12, ±2 KB); out-of-range → inverted branch over `j` (relaxation). |
| `Mov { dst, src }` | same shape | Wide `or dst, src, src` (what the assembler emits for `mov`). |
| `Load32 / Store32 / Store8 / Store16 { …, base, offset }` | same shape | `l32i`/`s32i`/`s8i`/`s16i`. Negative/out-of-range offsets fold into scratch (`base+off` then offset 0). |
| `Load8U / Load8S / Load16U / Load16S` | same shape | **Implemented + dual-run tested** (P6b, `tests/p6_emitter_completion.rs`): `l8ui`, `l16ui`, `l16si`; `Load8S` = `l8ui` + `sext …,7` (no `l8si` exists). Offsets gated per opcode (`0..=255` / `0..=510`×2), scratch fallback beyond. |
| `Neg / Bnot` | same shape | **Implemented + dual-run tested** (P6b): `neg` is a native RRR-form (`AluRt::Neg`); `bnot` = materialize −1 + `xor` (no not/xor-immediate — `ImmRule::NoImmForm`). |
| `MemcpyWords { dst_base, src_base, size }` | same shape | **Implemented + dual-run tested** (P6b): unrolled `l32i`/`s32i` pairs through scratch; copies past the 1020-byte offset ceiling bump both bases in ≤1024-byte chunks and restore them exactly (base regs observably unchanged — pinned by test). Word-multiple sizes only; size 0 and self-copy are no-ops. |
| `SlotAddr { dst, slot }` | same shape | `dst = a1 + slot_offset` (`addi`/`addmi`). Slot layout: see frame model below. |
| `IConst32 { dst, val }` | same shape | `movi` for −2048..=2047, else pooled `l32r`. |
| `Call { target: SymbolId, args: VRegSlice, rets, …sret flags }` | `Call { callee, args, ret: Option<PReg> }` **and** `CallMulti { callee, args, rets: Vec<PReg> }` | Post-regalloc: args/rets are physical regs. `Callee::Sym` is the real path (pooled absolute address + `callx8` — the monorepo links builtin addresses into the pool the same way); `Callee::Func` (PC-relative `call8` to a function in the same buffer) exists so call tests are position-independent and can dual-run on hardware. `CallMulti` is the full mirror (P4): **stack-passed args** past the 6-register capacity and up to 2 direct return words (caller `a10, a11`); the legacy `Call` keeps the M5 register-only contract pinned by the P1 arg-capacity study. See [Argument passing and returns](#argument-passing-and-returns-p4). |
| `Ret { vals: VRegSlice }` | `Ret { val: Option<PReg> }` and `RetMulti { vals: Vec<PReg> }` | `mov a2, val` + `retw`; two-word returns write `a2, a3` (`RET_REGS`) with clobber-free ordering (a `[a3, a2]` swap bounces through scratch). > 2 words refuse to emit — sret buffer instead. |
| `Edit::LoadIncomingArg { fp_offset, to }` (regalloc edit) | `IncomingStackArg { dst, index }` | Callee-side read of a stack-passed incoming arg: `l32i dst, a1, frame + 4*(index - 6)` — callee `SP + ENTRY frame` **is** the caller's SP. |
| `Label(LabelId, src_op)` | `Label(LabelId)` | Zero-size layout item. |
| `FuelCheck { vmctx, decrement, trap_label }` | `FuelCheck { fuel_base, decrement, trap_label }` | Full expansion, not a stub: `l32i scratch,[base]; beqz scratch→trap; (addi −1; s32i)`. `fuel_base` stands in for vmctx (the counter is the vmctx low fuel word); check-then-decrement ordering matches the real semantics. |

**Backport risk found: none structural.** Everything the emit stage needs stayed
expressible without lpvm-native types. The only genuinely-monorepo pieces are the ones
deliberately out of scope: `VReg`/regalloc, `VRegSlice` pools (become physical-reg
lists at emit), `ModuleSymbols` (becomes the pool's symbol-literal table), and the
sret/multi-return staging (a register-numbering detail under the windowed ABI, noted
above).

## Emitter policy (the durable contract)

1. **Pool-before-code.** Buffer layout is
   `[literal pool][func0: entry][func1]…`; `entry_offset` = pool size. `l32r` reaches
   literals **backward only** (`target = ((PC+3) & !3) + (one_extend(imm16) << 2)`,
   hardware-verified), so pool-at-start is reachable from the first ~256 KB of code and
   needs no mid-stream pool islands. The 16-bit field is **one-extended** (value =
   field − 0x10000): the full displacement range is −262144..=−4 bytes, and field
   values `0x0000..0x7FFF` reach the *farther* 128 KiB — the emitter's original assert
   used the sign-extended half range (−131072..), the bug the P6 imm-table work found;
   fixed and boundary-tested (`tests/p6_emitter_completion.rs`, fields
   `0x8000`/`0x7FFF`/`0x0000` at −131072/−131076/−262144, refusal at −262148). Literals are deduplicated by value (symbol slots
   by id) *within the buffer we own* — the spike showed assembler output dedups across
   an object and is therefore not self-contained; the emitter owning the pool is what
   makes the buffer relocatable as a unit.
2. **Position independence by construction.** Branches, `j`, and `call8` are
   PC-relative; pooled constants are values. The only absolute construct is a
   `Callee::Sym` address slot, reported in `EmitOut::sym_slots` for the host to patch
   after it learns the load address (on-device, the runner picks a heap address the
   host cannot know in advance — FINDINGS GV3).
3. **Branch fixups by iterative relaxation.** Label branches are layout items;
   sizing iterates (alignment padding depends on offsets, conditional branches are
   short 3-byte `beqz`/`bnez` until their ±2 KB range fails, then relax to the
   6-byte inverted-branch-over-`j` form). Relaxation is monotonic short→long, so the
   loop converges.
4. **Wide forms only** for lowered code (density optimization deferred); narrow
   `nop.n` appears only in function-entry alignment padding. `call8` targets are
   4-aligned (`(PC & !3) + (off << 2) + 4`), so function entries are padded to 4 with
   executable nops.
5. **Windowed ABI / frame model** (hardware-proven, FINDINGS E3/E4): one
   `entry a1, frame` prologue, `retw` epilogue; argument in `a2`, result in `a2`.
   Program registers `a2..=a7` (preserved across our calls by the rotation);
   `a8`/`a9` emitter scratch; `a10..=a15` outgoing args (callee's `a2..=a7`), result
   back in `a10`. Frame, from `a1+0` upward: **outgoing stack-arg area** (P4; sized
   for the call with the most stack-passed args, 0 when none), then stack slots, then
   the 32 reserved bytes at the top (16-byte base save area + 16 for `a4..a7` spills
   under call8), rounded to 16. **Large frames**: `entry`'s immediate caps at 32760
   bytes (imm12 × 8); with 16-byte rounding the largest emittable frame is **32752**
   (slot + out-arg bytes ≤ 32720). Beyond that the emitter raises a **documented hard
   error** naming the limit — deliberately *not* the `movsp` dynamic-frame idiom
   (policy pinned in the P2 ADR: LPIR has no alloca and no frame that large; `movsp`
   would add save-area-relocation semantics for an unreachable case). Both sides of
   the boundary are tested (`p6_entry_frame_boundary`: 32752 emits and runs, 32768
   refuses; `lp_xt_inst::encode` would have silently truncated to `entry a1, 0`).
   All of these register numbers live in the
   [ABI contract module](#abi-contract-srcgprrs-srcabirs) — the emitter contains
   no inline register numbers — and every immediate field is gated through the
   [legality table](#immediate-legality-srcimmrs) before encoding.
6. **Call increment: CALL8** (`abi::CALL_INC`, the single source of truth —
   `CallInc::default()` derives from it), still a parameter (`CallInc`,
   `emit_program_with`) so the same program can be emitted under CALL4/8/12.
   The choice is measurement-backed — preserved-register / register-arg /
   window-overflow tradeoffs on emulator and silicon, including CALL12's 2-arg
   ceiling and CALL4's arg-staging hazard: see
   [docs/call-inc-study.md](docs/call-inc-study.md) (P1 study;
   `tests/call_inc_study.rs` is the rig) and the ADR
   `docs/adr/2026-07-28-xtensa-abi-contract.md` (P2 decision).

## ABI contract (`src/gpr.rs`, `src/abi.rs`)

The hardware-validated register model, in the exact shapes the existing
`lpvm-native` register allocator consumes. **Backport mapping: `src/gpr.rs` →
`lpvm-native/src/isa/xt/gpr.rs`, `src/abi.rs` → `lpvm-native/src/isa/xt/abi.rs`**
— shape-for-shape mirrors of `isa/rv32/{gpr,abi}.rs`, so the move is a file copy
plus swapping this crate's `PReg` alias for lpvm-native's. The rv32 `abi.rs`
classification *functions* (`classify_params`/`classify_return`/`func_abi_*`)
depend on lpvm-native types and are written there against these constants.
Normative rationale: `docs/adr/2026-07-28-xtensa-abi-contract.md`.

Because a call rotates the register window, caller and callee see different names
for the same physical registers — every constant states its view; the views
differ by `CALL_ROTATION` (= 8 under CALL8, asserted in tests):

| Constant | Value | View / role |
|---|---|---|
| `RA_REG` / `SP_REG` | `a0` / `a1` | fixed roles; never allocatable |
| `FP_REG` | = `SP_REG` | **no frame pointer**: `ENTRY` fixes the frame, `a1` is stable, frames are fixed-size |
| `ARG_REGS` | `a2..=a7` (6) | **callee** view — incoming params, precolor targets |
| `OUT_ARG_REGS` | `a10..=a15` (6) | **caller** view — what `emit.rs` stages into |
| `RET_REGS` / `CALL_RET_REGS` | `a2,a3` / `a10,a11` | callee writes / caller reads |
| `SCRATCH` / `SCRATCH2` | `a8` / `a9` | emitter scratch (the non-staging caller-saved dead zone); excluded from the pool like rv32's t0–t2/t3 |
| `ALLOC_POOL` | `[a15..a10, a7..a2]` — **12 regs** | vs rv32's 13: near parity. Caller-saved front-loaded (rv32's LRU policy), both banks descending — see the module comment |
| `CALLER_SAVED_POOL` | `a10..=a15` | measured on silicon: caller `a_j` survives a CALL8 iff `j < 8` |
| `abi::CALL_INC` | `CallInc::Call8` | the P1/P2 policy decision |
| `abi::FRAME_TOP_RESERVED_BYTES` | **32** | the one new ISA hook `abi/frame.rs::compute()` needs (rv32 = 0): window save areas at the frame top |
| `abi::SRET_SCALAR_THRESHOLD` | 2 | same as rv32, deliberately — target-invariant LPIR return classification |
| `abi::STACK_ALIGNMENT` | 16 | mandated by the windowed ABI (save-area layout assumes it) |

Unlike rv32, the incoming-arg registers **are** pooled: rv32 excludes its arg
regs because they double as every call's outgoing staging; under CALL8 staging is
the separate `a10..=a15` bank, so `a2..=a7` are ordinary call-preserved
temporaries once the precolored params die (and their preservation is *free* —
window rotation, no prologue save/restore). `emit.rs` consumes these modules
exclusively; the `CallInc` arithmetic (`arg_base = 4·units + ARG_REGS[0]`, arg
capacity, save-area bytes) is derived from the same constants, and the dual-run
corpus staying green through the constants refactor is the proof the model
describes what already runs on silicon. Nothing in the contract is LX7-only —
every value carries to LX6 (classic ESP32) unchanged, verified on classic
silicon by the C3/C4 ladder and the P5 N-run corpus (zero divergences;
FINDINGS.md, "LX6 conformance").

## Argument passing and returns (P4)

The complete calling convention for calls that exceed the register contract,
validated dual-run (`tests/p4_stack_args.rs`) and **matched against the esp
toolchain as the behavioral oracle** — `fixtures/elf/call_conv.elf` (`many`,
`make_quad`, disassembled with `xtensa-esp32s3-elf-objdump`) — rather than
invented. No divergence from the oracle's convention was needed.

**Stack-passed arguments** (args 7+ under CALL8's 6-register capacity):

- The caller stores arg `i >= 6` at `[caller_SP + 4*(i - 6)]` — the **bottom
  of its own frame** (the outgoing-arg area, below its slots). Oracle: `main`
  stores args 7/8 of `many(…)` at `s32i a1, 0` / `s32i a1, 4`.
- The callee reads it at `[callee_SP + callee_frame + 4*(i - 6)]`, because
  `callee SP + ENTRY frame == caller SP`. Oracle: `many` (frame 32) reads
  `l32i a1, 32` / `l32i a1, 36`.
- This is the same region order as rv32's `abi/frame.rs::compute()`
  (`caller_arg_stack_size` at `[SP, SP+size)`, spills/slots above). The
  callee's own reserved save-area top (`[caller_SP - 32, caller_SP)`) sits
  directly below the caller's outgoing args — adjacent, never overlapping;
  the deep-recursion P4 case keeps stack args + sret buffers live while the
  window file wraps and spills around both.

**Returns**:

- 1–2 scalars: direct in callee `a2(, a3)` → caller `a10(, a11)` —
  `RET_REGS` / `CALL_RET_REGS`. Two-word returns are hardware-shape-tested
  including the full-swap staging case (`[a3, a2]` bounces through scratch).
- \> `SRET_SCALAR_THRESHOLD` (= 2) scalars: **sret buffer** — the caller
  allocates the buffer in its own frame (a slot) and passes its address as
  the **first** argument (callee `a2`); the callee stores through it and
  returns nothing in registers. Oracle: `make_quad` receives the pointer in
  `a2` and writes `a2+0..12`. Three direct return words refuse to emit
  (pinned by test) — the threshold is a hard boundary, not a soft preference.

**The `IsaTarget`-shaped rules** (what the monorepo's `isa/xt` arm returns —
pinned by `p4_isa_target_shaped_rules`):

| `IsaTarget` method | Xtensa value |
|---|---|
| `call_arg_reg_count()` | 6 (`ARG_REGS`, callee `a2..=a7`) |
| `call_arg_reg_hw(i)` | `ARG_REGS[i]` (precolor view; caller stages `OUT_ARG_REGS[i]`) |
| `direct_ret_reg_count()` | 2 (`RET_REGS` = `a2, a3`) |
| `sret_uses_buffer_for(n)` | `n > SRET_SCALAR_THRESHOLD` (= 2) |
| `lpir_call_stack_args_start(uses_sret, caller_passes_ptr)` | `6`, or `5` when `uses_sret && !caller_passes_ptr` (the emitter-injected sret pointer occupies `ARG_REGS[0]` — same formula as rv32 with 6 arg regs) |
| `lpir_call_arg_target_hw(…)` | rv32's sret-swap logic unchanged over `ARG_REGS` — the window rotation is invisible to the slot mapping (the emitter translates callee-view `ARG_REGS[k]` to caller staging `OUT_ARG_REGS[k]` mechanically) |
| `stack_alignment()` | 16 |

Plus the frame hook: `abi/frame.rs::compute()` places the outgoing-arg area
at `[SP, SP+size)` exactly as on rv32; the only Xtensa addition remains
`FRAME_TOP_RESERVED_BYTES = 32` at the frame top.

## Immediate legality (`src/imm.rs`)

The per-opcode immediate-legality table — the data input for parameterizing
`lpvm-native/src/imm.rs` by ISA. RV32 has one uniform story (imm12 everywhere);
Xtensa's legality is **per-opcode**, so the table is keyed by immediate-operand
class (`ImmOp`), each entry carrying:

- **`ImmRule`** — `Range { min, max, step }` (e.g. `addi` −128..=127; `addmi`
  multiples of 256 in ±32K; `l32i`/`s32i` offsets unsigned, ≤1020, ×4), a
  `Set` (the `b4const`/`b4constu` branch-compare lookup tables), or
  **`NoImmForm`** — the key Xtensa fact that `andi`/`ori`/`xori` do not exist
  (an explicit entry, not an omission: every bitwise-immediate must materialize
  the constant);
- **`PcRel`** — the base a displacement is measured from (`PC+4` for branches
  and `j`; `(PC&!3)+4` for `call0/4/8/12`; `(PC+3)&!3` for the backward-only
  `l32r`), so `is_legal` answers "can I reach this target?" in bytes, not raw
  field values;
- **`Fallback`** — the documented lowering when a value is illegal
  (`ConstThenReg`, `AddmiSplit`, `AddressScratch`, `InvertOverJ` relaxation,
  `IndirectViaL32r`, `OtherOpcode` — e.g. `srli` sa≥16 → `extui` — or
  `HardError`: never silent truncation; `lp_xt_inst::encode` masks fields and
  does not validate).

Provenance: ranges derived from the espressif/llvm-project Xtensa `.td` files
(Apache-2.0 WITH LLVM-exception, provenance header in the file), with every
boundary additionally probed against `xtensa-esp32s3-elf-as` (accepts min/max,
rejects one step beyond) and round-tripped through `lp-xt-inst` where encoded
(`tests/imm_legality.rs`, which also pins that the emitter's `add_imm`/`iconst`
thresholds match the table). Every rule is identical on LX6 — verified live by
`tests/imm_gas_lx6.rs`, which re-probes every boundary with both the LX6 and
LX7 assemblers (`xtensa-esp32-elf-as` / `xtensa-esp32s3-elf-as`) and requires
identical accept/reject verdicts and byte-identical encodings (171 cases,
zero differences; skips loudly when the espup toolchain is absent).

## Icmp branch table

| `IcmpCond` | Xtensa branch (branch-if-true) |
|---|---|
| Eq / Ne | `beq` / `bne` lhs, rhs |
| LtS / GeS | `blt` / `bge` lhs, rhs |
| LtU / GeU | `bltu` / `bgeu` lhs, rhs |
| GtS / LeS | `blt` / `bge` **rhs, lhs** |
| GtU / LeU | `bltu` / `bgeu` **rhs, lhs** |

## Validation

- Unit tests pin the golden-vector shape (the trivial function reproduces the spike's
  objdump-derived GV1 byte-for-byte), pool dedup + layout, entry alignment, relaxation,
  and sym-slot reporting; everything emitted round-trips `lp-xt-inst::decode`.
- `tests/dual_run.rs` runs the program corpus — arithmetic (including pooled
  constants and the shift/div/mulh mix), counted loop, forward/backward branches, the
  10-condition icmp matrix, Select, stack-slot byte/half/word stores, local-function
  `call8` "builtin", depth-100 recursion both without and **with a live stack slot
  per frame** (window overflow/underflow through emitted frames — the slotted variant
  is the hardware check that slots stay clear of the save areas; spills asserted via
  the emulator tracer), FuelCheck loops, and a relaxed >2 KB conditional branch — on
  the emulator against Rust-computed answers, and on the S3 when `XT_DEVICE_PORT` is
  set. The `callx8`-through-a-pooled-symbol path runs emulator-only by design
  (absolute addresses are unknowable pre-load on the device; hardware call coverage
  comes from the position-independent `call8` case) and exercises the same sym-slot
  linking flow the monorepo will use for builtin addresses.
- `tests/p4_stack_args.rs` (P4) dual-runs the argument/return convention: the
  8-arg oracle shape (2 stack-passed, weighted so any slot/offset mixup changes the
  result), 2-word direct returns (including the swap-staging case), the vec4-shaped
  sret buffer, and a deep recursion (depths 0–30, past the measured overflow onset at
  6) that keeps stack args, sret buffers, and saved slots live while windows spill —
  with a tracer assertion that spills actually happen. Threshold refusals (3 direct
  return words) and the CALL12 3-args-via-stack escape are emit-level tests.
- `tests/p6_emitter_completion.rs` (P6b) dual-runs the extending loads (sign bits
  probed in every byte/half position), Neg/Bnot, MemcpyWords (1056 bytes = two
  chunks, base-restoration verified observably), and a 2 KiB frame; emit-level
  boundary tests pin the ENTRY frame ceiling (32752 runs / 32768 refuses with the
  documented error) and the L32R one-extended displacement rule at all four
  boundary fields.
- The oracle earns its keep: an early revision of the fuel test read a register never
  written on one path — invisible on the emulator (zeroed registers), caught
  immediately by real silicon.

## Provenance

Original code, written for this experiment.

- Machine encodings: entirely delegated to `lp-xt-inst` (see that crate's provenance:
  encoding data derived from the espressif/llvm-project Xtensa `.td` files,
  Apache-2.0 WITH LLVM-exception, vendored in `licenses/`).
- ABI, `L32R`/`CALL8` target formulas, and the frame/save-area model: Xtensa ISA
  Reference Manual + ESP32-S3 TRM semantics, verified on hardware by the E2–E4 spike
  experiments (FINDINGS.md) and this crate's dual-run suite.
- No GPL source (binutils, QEMU, GCC) was copied, transliterated, or adapted — see
  `docs/adr/2026-07-28-license-provenance-discipline.md`.
