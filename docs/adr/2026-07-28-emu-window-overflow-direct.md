# ADR: Model Xtensa window overflow/underflow directly in the emulator

- Status: accepted
- Date: 2026-07-28
- Deciders: Yona Appletree
- Relates to: milestone M3 (`lp-xt-emu`)

## Context

The Xtensa windowed-register ABI keeps a 16-register software window over a
64-entry physical register file. When the call chain is deeper than the file
holds (~8 call8 frames), a `CALL`/`ENTRY` would reuse physical registers still
belonging to a live ancestor. Real hardware raises a `WindowOverflow{4,8,12}`
exception whose handler (installed by `xtensa-lx-rt`) spills that ancestor's
registers to its stack ABI save area; the symmetric `WindowUnderflow` handler
reloads them on `RETW`.

`lp-xt-emu` must reproduce this exactly — the emulator is the debugging asset the
whole Xtensa backend rests on, and deep recursion / window pressure is precisely
where a code generator's bugs will hide. There are two ways to get it:

1. **Emulate the handler vectors**: install the real spill/reload assembly at the
   exception-vector addresses and drive the CPU through it on each overflow.
2. **Model the effect directly**: when a `CALL`/`ENTRY`/`RETW` crosses a
   live-window boundary, perform the spill/reload of the affected register block
   to/from the ABI save area in emulator code.

The reference handlers live in QEMU (GPL-2.0) and binutils/`xtensa-lx-rt`; the
repo license discipline forbids copying GPL source, and the handler vectors are
firmware artifacts, not part of the ISA the emulator is meant to model.

## Decision

**Model window overflow/underflow directly in the emulator; do not emulate the
handler vectors.**

- Overflow is detected at `ENTRY`: the new frame's 16-register window spans four
  `WindowBase` units; the `4-inc` low units are shared with the caller by
  design, and the high `inc` units are its out-registers. If the ring has
  wrapped so a live ancestor still occupies them, that ancestor is spilled
  *before the frame runs* (a later `CALL` would otherwise clobber it).
- Underflow is handled at `RETW`: if the frame being returned into is
  non-resident, its registers are reloaded.
- Spill/reload write/read the **real ABI stack save area**, so the memory effect
  matches hardware for `a0..a3` (`[callee_sp-16, callee_sp)`), which is what a
  window reload actually needs.
- The live call chain is tracked as an explicit **call-stack shadow**, not a
  per-`WindowBase` table — base is reused as the ring wraps and is therefore not
  a stable frame identity. Each frame's save area is located from its callee's
  stack pointer, recovered from the shadow rather than by walking the resident
  window as the hardware handler chain does (an emulator convenience; the
  observable bytes are identical).

## Consequences

- **License-clean**: implemented from the ISA manual's windowed semantics and
  validated behaviorally against hardware; no GPL handler source reproduced.
- **Correctness is verifiable against ground truth**: the M3 dual-run corpus runs
  `call8`/`call12` recursion past depth 60/100 on both the emulator and the S3
  and asserts identical results — so any overflow/underflow bug surfaces as an
  emu-vs-device diff, bisectable by depth. (Two real bugs were caught this way
  during M3: the overflow trigger was one frame too late, and a per-base save-area
  table was clobbered by base reuse — both fixed by the call-stack model above.)
- **Trade-off**: the exact placement of the *extra* register groups (`a4..`,
  `a8..`) differs from the hardware handlers' caller-relative layout. This is not
  observable for bare payloads (which never read another frame's save area) and
  is documented at the seam; if a future need arises (e.g. a guest that walks its
  own spill area), the placement can be made bit-exact.
- **Backport-ready**: the same seam lands in `lp2025` unchanged; only the trace
  layer grows toward `InstLog` parity.

## Alternatives considered

- **Emulate the handler vectors** (option 1): rejected — would either copy GPL
  handler source or require re-deriving firmware assembly the emulator has no
  reason to model, for no fidelity gain on any observable the runner can report.
