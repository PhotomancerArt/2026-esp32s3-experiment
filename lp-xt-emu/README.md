# lp-xt-emu

A pure-Rust Xtensa (ESP32-S3 / LX7) instruction-set **emulator core** with the
windowed-register machinery. Part of the standalone Xtensa-backend work
(milestone M3); mirrors `lp2025`'s `lp-riscv-emu` architecture so the eventual
monorepo backport is a merge, not a rewrite.

Scope is *core*: executors + memory + the window machinery. FPU, cycle model,
peripherals, and full `InstLog` parity are out of scope (the trace layer is a
hook now, built out during backport against the real filetest consumers).

## Architecture

```
src/
  cpu.rs         CPU state: PC, 64 physical ARs, WindowBase, WindowStart, SAR,
                 PS.CALLINC, and the live call-stack shadow.
  memory.rs      Vec-backed regions + the SRAM1 D-bus/I-bus dual mapping.
  trace.rs       `trait Tracer` (no-op default) + a basic text tracer.
  error.rs       Trap { Exception | Timeout }, mirroring CrashReport.
  emu.rs         Emulator: fetch/decode/execute loop + the windowed-ABI run API.
  executor/      one module per instruction group (the lp-riscv-emu split):
                 arith · imm · load_store · branch · jump · call · window · misc
```

Decoding is delegated to [`lp-xt-inst`](../lp-xt-inst); this crate never
re-implements it. Instruction semantics come from the Xtensa ISA Reference
Manual and are validated by diffing against real hardware (see below).

### The windowed register view

The LX7 has a 64-entry *physical* address-register file. Software sees a
rotating 16-register window; `a{i} == AR[(WindowBase*4 + i) mod 64]`.
`WindowStart` bit `k` marks a live call frame based at `WindowBase == k` whose
registers are currently *resident* (not spilled).

- **CALL8/CALLX8** (and call4/12) do **not** rotate. They stage the return
  address — with the call-increment in the top two bits — into the caller's
  `a[4*inc]`, and record `PS.CALLINC`.
- **ENTRY** rotates `WindowBase` forward by `PS.CALLINC`, allocates the stack
  frame, and sets the new `WindowStart` bit. The caller's `a10..` become the
  callee's `a2..` — this is how `f(arg)` receives `arg`.
- **RETW** rotates back by the increment recorded in `a0`'s top two bits and
  unmangles the return PC (`(PC & 0xC000_0000) | (a0 & 0x3FFF_FFFF)`).

### Window overflow / underflow — modeled directly

When the register ring wraps so a new frame's registers would overwrite a still
live ancestor, the ancestor is **spilled** to its ABI stack save area, and
**reloaded** on the return path — the effect of the `_WindowOverflow` /
`_WindowUnderflow` handlers, implemented directly rather than by emulating the
handler vectors. See
[`docs/adr/2026-07-28-emu-window-overflow-direct.md`](../docs/adr/2026-07-28-emu-window-overflow-direct.md).

The frame chain is tracked as an explicit call-stack shadow (not a per-base
table): `WindowBase` is reused as the ring wraps, so it is *not* a stable frame
identity. A frame's base save area (`a0..a3`) is located from its **callee's**
stack pointer at `[callee_sp-16, callee_sp)`, exactly as the hardware handler
chain recovers it by walking the resident window — so a spill and its later
reload always address the same bytes. Extra register groups (`a4..`, `a8..`) for
call8/call12 frames are placed just below; their exact byte placement is not
observable for bare payloads (which never read another frame's save area), the
deliberate "model the effect, not the handler vectors" boundary.

### SRAM1 D-bus / I-bus dual mapping

ESP32-S3 SRAM1 is dual-mapped: a byte written at a D-bus address
(`0x3FC8_8000..0x3FCF_0000`) is fetchable at the I-bus alias `+0x6F_0000`. The
`xt-runner` firmware writes payloads via D-bus and *executes them at the I-bus
alias*, so self-addressing code (`l32r` literals, `call8` targets) only behaves
identically if the emulator models the same alias. `memory.rs` backs the dual
mapping with one store reachable at both address ranges; fetch is permitted only
at the executable (I-bus) view, so jumping to a D-bus address faults exactly as
hardware does (FINDINGS E2D).

## Run API

`Emulator::run(code, entry_offset, arg)` loads `code` into SRAM1 and invokes it
exactly as the device runner does — a synthesized windowed `CALL8`, `arg` staged
in `a10` and arriving in the callee's `a2` after its `ENTRY` — returning
`RunOutcome::Ok(result)` or `RunOutcome::Trap`. `run_traced` additionally emits
`TraceEvent`s (per-instruction, register/memory writes naming the physical AR,
and window rotate/spill/reload events).

## Validation — dual-run against hardware

`tests/conformance.rs` runs every corpus case on the emulator, and — when
`XT_DEVICE_PORT` is set — on the S3 via [`xt-runner-client`](../xt-runner-client),
asserting equal results and equal crash classification.

```bash
cargo test -p lp-xt-emu                                        # emu-only (known-answer)
XT_DEVICE_PORT=/dev/cu.usbmodem1101 \
  cargo test -p lp-xt-emu -- --test-threads=1 --nocapture      # dual-run vs hardware
```

Hardware tests **must** run single-threaded (`--test-threads=1`) — there is one
board, shared. Flash the runner first (`cd ../fw/xt-runner && cargo run
--release`).

Corpus: golden vectors GV1–GV3b plus a generated set — arithmetic, load/store
round-trips, branches both directions, a backward-branch loop, `call8` and
`call12` self-recursion past depth 16/60/100 (the key window-overflow/underflow
stress), a `callx8`-to-builtin case, and illegal-instruction / hang faults. The
recursion blobs use PC-relative `call8`/`call12` so they are position-independent
and dual-runnable; the `callx8` + `l32r` golden vectors self-address via absolute
literals and so are emulator-only known-answer.

All payload bytes are objdump-derived from toolchain-assembled sources, never
hand-recalled (repo lesson: 2/3 hand-recalls are wrong).

## Provenance

**This is original code.** Instruction semantics and the windowed-register model
are implemented from the **Xtensa ISA Reference Manual** and validated
behaviorally against real ESP32-S3 hardware (the `xt-runner` oracle). Encoding
data is consumed via `lp-xt-inst` (whose provenance derives from the Apache-2.0
LLVM Xtensa tables).

**No GPL source was used.** QEMU (`espressif/qemu`) and binutils/GDB — including
their windowed-register handling and the `_WindowOverflow`/`_WindowUnderflow`
handlers — are behavioral references only: observed to understand semantics,
never copied or transliterated. See the repo license ADR
(`docs/adr/2026-07-28-license-provenance-discipline.md`) and `AGENTS.md`.
