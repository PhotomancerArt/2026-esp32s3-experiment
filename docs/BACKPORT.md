# Backport seam: from `2026-esp32s3-experiment` into lp2025

This document maps the standalone Xtensa core built here onto the lightplayer
monorepo (`lp2025`). It is the **experiment-side** half of the backport picture;
the **monorepo-side** half — what prep lp2025 itself needs — lives in that repo's
`docs/reports/2026-07-28-xtensa-monorepo-readiness.md`. Read both together.

Everything below is validated on real ESP32-S3 hardware (see `FINDINGS.md` and
each crate's README). The two hardest risks of the whole Xtensa project — the
windowed-register ABI and the espup toolchain — are retired: the emulator matches
silicon through depth-100 window-overflow recursion, and an emitter built on
`lp-xt-inst` produces code that agrees emu-vs-device across a 60-case corpus.

## Naming convention (established here)

- `lp-xt-*` = destined for the monorepo backport.
- bare `xt-*` = experiment-local scaffolding (its *learnings* port; its code
  mostly stays here).

## Per-artifact landing table

| Here | Lands in lp2025 as | Notes |
|---|---|---|
| `lp-xt-inst` | `lp-xt/lp-xt-inst` (sibling of `lp-riscv/lp-riscv-inst`) | Drop-in. `no_std`, `forbid(unsafe_code)`, zero deps. Its `format_instruction` feeds filetest asm snapshots + `shader-debug`. Derived from LLVM `.td` with provenance headers — keep them. |
| `lp-xt-emu` | `lp-xt/lp-xt-emu` (sibling of `lp-riscv/lp-riscv-emu`) | **Depends on the `lp-emu-core` extraction** (ARM report's refactor #2, still not done in lp2025 — see readiness report). What's stubbed here as hooks, to be built there against real consumers: the full `InstLog`/trace parity layer and a cycle model (`--perf`). The window machinery, memory model, and executors port as-is. |
| `lp-xt-elf` | `lp-xt/lp-xt-elf` | Linked-exec loader ports directly. Its **reloc engine** (M6) is what the monorepo's builtins-object link path needs; see the M6 verdict below. |
| `fixtures/lp-xt-emu-guest` | `lp-xt/lp-xt-emu-guest` → feeds a future `fw-emu-xt` | Guest trap ABI (SYSCALL nr in a2; 1=EXIT/2=WRITE/3=PANIC). Mirror of `lp-riscv-emu-guest`. |
| `xt-mini-emit` | **NOT a crate** → `lpvm-native/src/isa/xt/` (`encode.rs`, `emit.rs`, `abi.rs`, `gpr.rs`, `link.rs`), sibling of `isa/rv32/` | The emitter *logic* ports; MiniVInst is replaced by the real `VInst`. See the mapping section. |
| `xt-runner` + `xt-runner-client` + `xt-runner-proto` | **stay experiment-local** | The on-device payload runner is the fast inner loop + hardware oracle. Strong candidate for the **tethered-S3 CI conformance rig** the roadmap wants (open gate question). |

## `xt-mini-emit` → `isa/xt/` port (the emitter)

Backport finding (from M5, hardware-verified): **MiniVInst stayed a faithful
structural mirror of `lpvm-native`'s `VInst` with no dependency on lpvm-native
types.** So the port is mechanical substitution, not redesign. The mapping table
in `xt-mini-emit/README.md` is authoritative; the shape changes are:

- `PReg` → the real allocator's physical-register output (`Alloc::Reg`). MiniVInst
  already assumes allocation is done, matching the emit stage.
- `MiniVInst`'s inline operand vectors → the real `VRegSlice`/`ModuleSymbols` pools.
- `Callee::{Func, Sym}` split → re-unify as the real single `Call{SymbolId}` form
  emitting a literal-slot relocation. `link_syms` in the M5 tests is exactly that
  flow.
- Unmirrored-but-mechanical variants to add during the port (documented per-row in
  the README): `Neg`, `Bnot`, `Load8U/8S/16U/16S`, `MemcpyWords`, sret ABI flags.

**Emitter policy contract to carry over (all hardware-proven):** pool-before-code
buffers, backward-only `L32R` targeting `((PC+3)&~3)+(imm16<<2)` with in-buffer
literal dedup (the emitter must own pools — LLVM MC's cross-object dedup was the
spike's warning); windowed `ENTRY`/`RETW` frames; register model a0–a7 preserved /
a8–a15 scratch / args a10+ / return a10→a2; iterative branch relaxation
(`beqz`/`bnez` past ±2KB → inverted-branch-over-`J`); wide instruction forms first
(density is a later optimization).

## `IsaTarget::Xtensa` dispatch checklist (monorepo)

The monorepo readiness report enumerates the exact match-arm sites; from the
experiment side, the dispatch surface a new `IsaTarget::Xtensa` arm must satisfy
lives in `lpvm-native/src/{compile.rs, emit.rs, abi/func_abi.rs, regalloc/walk.rs,
rt_emu/engine.rs, rt_jit/module.rs}` plus `lpc-hardware/.../hw_target.rs`. The
readiness report's count (≈19 arms, ~6 remaining hardcodes) is the number to work
against. Two sleepers it flags that this experiment corroborates:

- **Immediate legality** is per-opcode on Xtensa and *unlike* RV32 — notably there
  is **no `ANDI`/`ORI`/`XORI`** (M5 handles bitwise-immediate via a pooled constant
  + register op). `imm.rs` must become ISA-parameterized.
- **rt_jit buffer needs a write/exec pointer split**: on S3 you write at the D-bus
  address and execute at the `+0x6F0000` I-bus alias (proven in E2/`jitbuf.rs`).

## Filetest integration sketch

New targets `xtn.q32` (lpvm-native + lp-xt-emu) and `xtlpn.q32` (lps-glsl frontend
+ same), mirroring `rv32n.q32`/`rv32lpn.q32`. `lps-filetests` currently hard-imports
`lp_riscv_emu` types (`LogLevel`, `CycleModel`) in `test_run/run_detail.rs` and
`perf_model.rs` — those become the `lp-emu-core` interface that both `lp-xt-emu` and
`lp-riscv-emu` implement. The 851 target-agnostic `.glsl` filetests become the
Xtensa conformance corpus for free.

## License provenance (carry into lp2025)

Every derived file here (`lp-xt-inst` from LLVM `.td`) carries a provenance header;
the license is vendored in `licenses/`. **Mirror `docs/adr/2026-07-28-license-
provenance-discipline.md` into lp2025's `docs/adr/`** — lp2025 is where outside
(AGPL) contributions actually arrive, so the no-GPL-source discipline must govern
its Xtensa crates too. The two GPL clones (`oss/qemu-xtensa`, `oss/binutils-gdb`)
were behavioral references only; no GPL source was copied into any crate.

## M6 — `.o` relocation verdict

**Done, and cheaper than budgeted.** The relocation engine (feature-gated in
`lp-xt-elf`, base crate unaffected) implements `R_XTENSA_32` and — the item the
original analysis flagged as "the real pain" — **`R_XTENSA_SLOT0_OP`, working**
across `call0/4/8/12`, `j`, `l32r`, RRI8/BRI12 branches, and narrow `beqz/bnez`.

The verdict for the monorepo estimate: **the builtins-object link path is tractable
and cheap.** SLOT0_OP reduces to ~60 lines (`retarget_slot0`) *because the hard part
was already paid in M2* — `lp-xt-inst` decodes/re-encodes every slot format, so the
reloc just recomputes the PC-relative operand and re-encodes. Three two-object
fixtures (cross-object `call8`, function-pointer/`.data`/`.bss` via `R_XTENSA_32`,
bidirectional calls) run correctly on the emulator, each cross-checked against GNU
ld's own linked output as a behavioral oracle (field math validated against ld's
patched bytes). No binutils source was read into the implementation — semantics from
the psABI numbering + the ISA PC formulas already in `lp-xt-inst` + diffing ld/gas
*output*.

Two caveats to carry into the real linker: it must handle (or keep discarding)
`R_XTENSA_DIFF*` debug/`.xt.prop` sections, and `.literal` placement order is a
backward-only-`l32r` layout invariant the linker must own.

## What is NOT proven here (carry as monorepo risk)

- FPU executors (fixtures + emitter are integer-only; lightplayer's device path is
  Q32, but `fw-emu-xt` running arbitrary fork-compiled code will need FPU in the emu).
- Full `InstLog`/trace parity + cycle model (`lp-xt-emu` has hook points only).
- The `lp-emu-core` extraction itself (monorepo refactor #2 — the prize is ~2× bigger
  now that `profile/` landed in lp-riscv-emu; see readiness report).
- Classic ESP32 (LX6) — S3 only. Its IRAM/word-access JIT model and RAM footprint are
  the fast-follow's open questions.
- Absolute-symbol `CALLX8` on device (emulator-only here; PC-relative `CALL8` is the
  device-proven path; a runtime-base-discovery probe worked but is fragile — see M5).
- `lp-xt-emu` returns 0 on divide-by-zero; hardware raises `IntegerDivideByZero`
  (unexercised by the corpus).
