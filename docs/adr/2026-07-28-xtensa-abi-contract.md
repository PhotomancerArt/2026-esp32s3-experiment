# ADR: Xtensa ABI contract — CALL8 windowed convention and register model

- Status: accepted
- Date: 2026-07-28
- Deciders: Yona Appletree
- Relates to: P1/P2 of the compiler-contract plan; `xt-mini-emit/src/{gpr,abi}.rs`
  (the normative constants), `xt-mini-emit/docs/call-inc-study.md` (the measurements)

## Context

lightplayer's native shader backend (`lpvm-native`) compiles LPIR through one
register allocator that is parameterized per ISA: `IsaTarget` hands it plain
hardware register numbers (`allocatable_pool_order()`, `caller_saved_pool_hw()`,
`call_arg_reg_hw()`, …) and the `abi/frame.rs::compute()` frame layout is driven by
a handful of constants. Porting to Xtensa (ESP32-S3) therefore does not need a new
allocator — it needs a **register model in those exact shapes**, with values that
are true on silicon.

Xtensa's windowed ABI is the part that looks like it might break this: a `CALLn`
instruction rotates a 16-register window over a 64-entry physical file (callee
`a0` = caller `a[n]`), and when call depth exhausts the file, hardware traps spill
ancestor windows to per-frame stack save areas. The standalone-core work
established (and the P1 study re-confirmed on hardware) that **all of this is
invisible to register allocation**: from the allocator's seat, Xtensa is a
conventional 16-register machine with fixed roles, a caller/callee-saved split,
and one extra frame-layout rule. The window machinery lives below the ABI line,
in hardware and trap handlers.

What the ABI line *does* have to fix is the **call increment** — CALL4, CALL8, or
CALL12 — because the rotation distance simultaneously determines:

- how many caller registers survive a call (caller `a_j` survives iff `j < n`),
- how many arguments fit in registers (callee always reads `a2..=a7`, but the
  caller can only stage inside its own window), and
- how fast deep call chains start paying window-overflow traps.

The P1 study measured all three for every increment, on the emulator and on real
ESP32-S3 silicon (which agreed everywhere — 42/42 survival probes, 120/120
recursion points, every arg-capacity case). Numbers below are from that study.

## Decision

### 1. Call increment: CALL8 (`CALL8`/`CALLX8` for every emitted call)

Measured comparison (`xt-mini-emit/docs/call-inc-study.md`):

| | CALL4 | **CALL8** | CALL12 |
|---|---|---|---|
| Registers preserved across a call | 2 (`a2,a3`) | **6 (`a2..a7`)** | 10 (`a2..a11`) |
| Register-argument capacity | 6 | **6** | **2** — 3-arg calls cannot be emitted |
| Arg staging | overlaps program regs `a6/a7` (permanent emitter hazard) | **conflict-free** | conflict-free |
| First window-spill depth / regs spilled at depth 40 | 11 / 124 | 6 / 280 | 4 / 436 |
| Frame save-area reservation | 32 B (uniform floor) | 32 B | 48 B |

- **CALL12 is disqualified** by the 2-register-arg ceiling: the callee's `a2..a7`
  map to caller `a14..a19` and the caller's window ends at `a15`. A 3-argument
  call has no emission at all (pinned by test), and LPIR calls routinely carry
  3+ args — CALL12 would force stack args onto *common* calls. It also traps
  earliest and moves the most bytes per trap.
- **CALL4 is disqualified** by its 2 surviving registers: the allocator would
  spill around every call to protect 4 of its 6 program registers, a common-path
  cost bought back only in call chains deeper than ~6 frames, which LPIR shader
  code (shallow, builtin-heavy) does not have. It also carries the `a6/a7`
  staging-overlap special case in the emitter forever.
- **CALL8 has no disqualifier**: 6 preserved temporaries (near rv32's pool
  economics), 6 register args, disjoint staging, mid-pack trap onset, and the
  32-byte frame reservation already hardware-proven by the M5 corpus.

### 2. Register model (normative constants: `xt-mini-emit/src/gpr.rs`)

Because the window rotates at each call, caller and callee see different names
for the same physical registers; every constant states its view. The two views
differ by the CALL8 rotation, `+8`.

| Role | Value | View |
|---|---|---|
| `RA_REG` | `a0` | — (written mangled by `CALLn`; consumed by `RETW`) |
| `SP_REG` | `a1` | — (`ENTRY` establishes it; stable per frame) |
| `FP_REG` | = `SP_REG` | **no frame pointer** — see below |
| `ARG_REGS` | `a2..=a7` (6) | callee (incoming params; precolor targets) |
| `OUT_ARG_REGS` | `a10..=a15` (6) | caller (outgoing staging) |
| `RET_REGS` | `a2,a3` | callee (what `Ret` writes) |
| `CALL_RET_REGS` | `a10,a11` | caller (where a call's result lands) |
| `SCRATCH`, `SCRATCH2` | `a8`, `a9` | emitter scratch, never allocatable |
| `ALLOC_POOL` | `[a15..a10 desc, a7..a2 desc]` — **12 registers** | allocator |
| `CALLER_SAVED_POOL` | `a10..=a15` (6) | clobbered by a call (measured) |

- **No frame pointer.** rv32 dedicates s0 because its prologue moves SP and
  large/dynamic frames want a fixed base. Under the windowed ABI, `ENTRY`
  establishes the frame in one instruction, frames are fixed-size (no alloca in
  LPIR; frames past `ENTRY`'s 32760-byte immediate are a hard error rather than
  the `movsp` idiom), and `a1` is invariant for the frame's lifetime. All
  addressing is SP-relative; `FP_REG` aliases `SP_REG` for shape parity.
- **Scratch is `a8`/`a9`** because they are the only caller-saved registers that
  are not argument staging: `a8` is where CALL8 writes the (CALLINC-mangled)
  return address, and `a9` sits in the same dead zone below the staging area —
  nothing can be live there across a call, so reserving them costs zero pool.
- **The pool is 12 registers vs rv32's 13 — near parity, not halved.** The
  windowed file does not shrink the allocator's world. Unlike rv32, the incoming
  arg registers *are* pooled: rv32 excludes its arg regs because they double as
  every call's outgoing staging; under CALL8 outgoing staging is the separate
  `a10..=a15` bank, so after the precolored parameters die, `a2..=a7` are
  ordinary call-preserved temporaries. A 16-register window can't afford an
  unpooled 6-register arg bank, and doesn't need one.
- **Pool order** (= LRU init order) front-loads the caller-saved bank, mirroring
  rv32's policy (short-lived values land where calls clobber them for free).
  Both banks run descending: staging fills upward from `a10`, so handing out
  `a15` first keeps the slots every call uses free longest; `a2/a3` are the
  return/first-param registers, so keeping them free longest makes return moves
  no-ops more often.
- **Preservation is free.** rv32 pays prologue save/restore for its 10
  callee-saved pool members; Xtensa's `a2..=a7` survive by rotation at no
  instruction cost — the price is amortized window-overflow traps (measured:
  onset at call depth 6, +1 trap per frame steady-state).
- The caller-saved split is the **silicon-measured** survival rule: caller `a_j`
  survives a CALL8 iff `j < 8`; `a8` is destroyed by the call itself, `a9` by
  the callee's `ENTRY`.

### 3. ABI constants (normative: `xt-mini-emit/src/abi.rs`)

- `SRET_SCALAR_THRESHOLD = 2` (same as rv32, deliberately): rv32's value matches
  Cranelift's `signature_for_ir_func`; keeping it makes LPIR return
  classification (vec2 direct, vec3/vec4 sret) target-invariant across ISAs, so
  filetests and call lowering behave identically. Two direct return words stay
  inside the proven register contract (callee `a2,a3` -> caller `a10,a11`). The
  windowed ABI would permit 4; widening buys nothing LPIR needs and forks the
  classification.
- `STACK_ALIGNMENT = 16`: the Xtensa windowed ABI mandates 16-byte SP alignment,
  and the save-area layout assumes it. Same number as rv32 for the ABI's own
  reasons.

### 4. Frame layout: 32 reserved bytes at the top of every frame

The one genuine addition the monorepo's `abi/frame.rs::compute()` needs is an ISA
hook for **reserved bytes at the frame top**: `FRAME_TOP_RESERVED_BYTES = 32`
(rv32 = 0). The window trap handlers write this region *unbidden*: the 16-byte
base save area receives an ancestor's `a0..a3`, and `_WindowOverflow8` spills the
victim's `a4..a7` into the next 16 bytes — `16 × units`, so 32 under CALL8.
Everything else (slots, spills, outgoing stack args) builds upward from `SP+0`
exactly as rv32 does. Getting this wrong corrupts *ancestor* frames invisibly;
the M5/P1 recursion corpus (slotted recursion to depth 100, spill/reload
round-trips at every depth 1..=40) is the hardware proof of the layout, and P5
adds systematic torture.

### 5. Single source of truth

The emitter contains no literal register numbers: `emit.rs` consumes `gpr.rs` /
`abi.rs` exclusively, and the `CallInc` arithmetic (`arg_base = 4·units +
ARG_REGS[0]`, arg capacity, save-area bytes) is derived from the same constants.
Tests assert the cross-links (views differ by the rotation; the frame reservation
equals the policy's save-area rule; the caller-saved split matches the measured
survival predicate). The dual-run corpus staying green through the refactor is
the proof the constants describe what already runs on silicon.

## Consequences

- **The lp2025 backport is configuration, not design**: `xt-mini-emit/src/gpr.rs`
  and `src/abi.rs` move to `lpvm-native/src/isa/xt/{gpr,abi}.rs` with the crate's
  `PReg` alias swapped for lpvm-native's; the existing allocator is pointed at
  `ALLOC_POOL`/`CALLER_SAVED_POOL` unchanged. `RegPool`'s 32-entry table simply
  leaves indices 16–31 unused.
- `abi/frame.rs::compute()` gains one parameter (top-of-frame reservation,
  rv32 = 0) — a hook, not a redesign.
- Register pressure is near-rv32 (12 vs 13); the earlier "windowed ABI halves the
  pool" worry is retired by measurement. P5 measures achieved pressure for real.
- Multi-word returns beyond 2 use sret, identical to rv32 — no new classification
  paths. P4 hardware-validates stack args, sret, and 2-word returns.
- Staging conflicts for pooled `a10..=a15` members around calls are the
  allocator's existing caller-saved handling (save/restore live vregs) — the same
  contract rv32's `t4..t6` already exercise.
- **LX6 (classic ESP32) compatibility**: every choice in this contract is core
  windowed-ABI machinery — CALL8/ENTRY/RETW, the 64-register file, the save-area
  layout, and all immediate ranges are identical on LX6. Nothing here would
  differ for the classic-ESP32 fast-follow; the S3-specific parts of the repo
  (memory map, I/D-bus aliasing) are runner concerns, not ABI concerns.

## Alternatives considered

- **CALL4 / CALL12 increments**: rejected on measurement, not taste — see the
  table above (CALL12's un-emittable 3-arg calls; CALL4's 2 survivors + staging
  hazard). Recorded in full in `xt-mini-emit/docs/call-inc-study.md`.
- **Mixed increments** (e.g. CALL12 for leaf-heavy regions, CALL8 elsewhere):
  rejected — the preserved-set and staging rules would become per-call-site,
  which *does* leak into register allocation (the exact complexity this contract
  exists to avoid), for at most +2 temporaries in special cases.
- **A dedicated frame pointer**: rejected — no dynamic frames, `a1` is stable,
  and a 16-register window cannot spare a register with no job.
- **Excluding `a2..=a7` from the pool (strict rv32 mirroring)**: rejected — it
  would leave a 6-register pool and waste the windowed ABI's free preservation;
  the reason rv32 excludes its arg regs (they are every call's staging area)
  does not apply under CALL8.
- **`SRET_SCALAR_THRESHOLD = 4`** (the windowed ABI's full direct-return width):
  rejected — diverges LPIR-level classification between targets for no need
  LPIR has; revisit only if profiling shows vec3/vec4 sret traffic matters.
