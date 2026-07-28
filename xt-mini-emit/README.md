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
| `Load8U / Load8S / Load16U / Load16S` | *not mirrored* | Same pattern as Load32 (`l8ui`, `l16ui`, `l16si`; `Load8S` = `l8ui` + `sext …,7`). Omitted from the mini set; nothing new to prove. |
| `Neg / Bnot` | *not mirrored* | Trivial: `neg` exists as an RRR-form (`AluRt::Neg`); `bnot` = materialize −1 + `xor`. |
| `MemcpyWords` | *not mirrored* | A loop over `l32i`/`s32i`; composed of pieces proven here. |
| `SlotAddr { dst, slot }` | same shape | `dst = a1 + slot_offset` (`addi`/`addmi`). Slot layout: see frame model below. |
| `IConst32 { dst, val }` | same shape | `movi` for −2048..=2047, else pooled `l32r`. |
| `Call { target: SymbolId, args: VRegSlice, rets, …sret flags }` | `Call { callee: Callee, args: Vec<PReg>, ret: Option<PReg> }` | Post-regalloc: args/ret are physical regs. `Callee::Sym` is the real path (pooled absolute address + `callx8` — the monorepo links builtin addresses into the pool the same way); `Callee::Func` (PC-relative `call8` to a function in the same buffer) exists so call tests are position-independent and can dual-run on hardware. The sret-plumbing flags don't apply to the single-scalar mini ABI — **backport note**: multi-value returns land in `a10, a11, …` (callee `a2, a3, …`); nothing about the window rotation changes. |
| `Ret { vals: VRegSlice }` | `Ret { val: Option<PReg> }` | `mov a2, val` + `retw`. Multi-value: `a2..`. |
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
   literals **backward only** (`target = ((PC+3) & !3) + (imm16 << 2)`,
   hardware-verified), so pool-at-start is reachable from the first ~256 KB of code and
   needs no mid-stream pool islands. Literals are deduplicated by value (symbol slots
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
   back in `a10`. Frame = 32 reserved bytes at the top (16-byte base save area +
   16 for `a4..a7` spills under call8) + stack slots growing from `a1+0`, rounded to
   16. `entry`'s immediate caps at 32760 bytes; larger frames need the `movsp` path
   (not needed here — asserted).

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
