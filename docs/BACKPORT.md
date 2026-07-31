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

## Register model & ABI contract (the compiler-contract phase)

**The headline for whoever does the backport: the windowed ABI is INVISIBLE to register
allocation. Use lpvm-native's existing allocator — configure it, do not rewrite it.**

Evidence: the entire `IsaTarget` surface (`isa/mod.rs`) returns plain hardware register
numbers and counts — `allocatable_pool_order() -> &[u8]`, `caller_saved_pool_hw()`,
`call_arg_reg_hw(idx)`, `direct_ret_reg_count()`, `sret_uses_buffer_for()`,
`stack_alignment()`, `lpir_call_arg_target_hw()`, `lpir_call_stack_args_start()`. Nothing
window-shaped. Rotation and overflow/underflow spilling happen *below* the ABI line
(hardware + runtime handlers). From the allocator's seat, Xtensa is a conventional
machine.

Corollaries for the port:
- `RegPool.preg_vreg: [Option<VReg>; 32]` (`regalloc/pool.rs`) is **fine as-is**: it is
  indexed by hardware number and the LRU is driven by `allocatable_pool_order()`, so a
  16-register ISA simply leaves indices 16–31 `None`. The `Alloc::Reg(u8) -> PReg`
  promotion is cleanliness, not a blocker.
- **The one structural change needed**: `abi/frame.rs::compute()` must gain an ISA hook
  for **reserved bytes at the TOP of the frame** — the window save areas that overflow
  handlers write unbidden. rv32 = 0; Xtensa = **32** (`FRAME_TOP_RESERVED_BYTES`). Get
  this wrong and ancestor frames corrupt silently; see the torture evidence below.

### The constants (land as `lpvm-native/src/isa/xt/{gpr,abi}.rs`)

Mirrors of `isa/rv32/{gpr,abi}.rs` shape-for-shape (`xt-mini-emit/src/{gpr,abi}.rs`):

```rust
RA_REG = a0        SP_REG = a1        FP_REG = SP (no frame pointer — ENTRY establishes
CALL_ROTATION = 8                      the frame; a1 invariant; a 16-reg window can't
SCRATCH = a8, SCRATCH2 = a9            spare a register with no job)
ARG_REGS      = [a2..a7]  (callee view — precolor targets)
OUT_ARG_REGS  = [a10..a15] (caller view — emit staging)
RET_REGS      = [a2,a3]   CALL_RET_REGS = [a10,a11]
ALLOC_POOL        = [15,14,13,12,11,10, 7,6,5,4,3,2]   // 12 registers
CALLER_SAVED_POOL = [15,14,13,12,11,10]
SRET_SCALAR_THRESHOLD = 2   STACK_ALIGNMENT = 16   FRAME_TOP_RESERVED_BYTES = 32
```

**Two views, not one** — the windowed subtlety that will otherwise cause silent
wrong-register bugs: argument/return constants exist in a *caller* and a *callee* flavor
differing by `CALL_ROTATION`. Cross-checked by tests.

**Pool size: 12 vs rv32's 13 — near parity, not the feared halving.** rv32 must exclude
its argument registers from the pool because they double as every call's outgoing
staging area; under CALL8 the rotation makes staging a *separate* bank (a10–a15), so
a2–a7 are ordinary call-preserved temporaries — and they are preserved **for free** by
the rotation, where rv32 pays prologue save/restore for its callee-saved members.
Measured in practice, not just by construction: live sets of 4/8/12 values compile with
**zero spill slots**; 16 → 4 slots; 20 → 8 slots.

### CALL-increment policy: CALL8 (measured, not assumed)

`CALLn` makes callee `a0` = caller `a[n]`; args are always callee a2–a7. Measured on
silicon (emulator agreeing):

| | CALL4 | **CALL8** | CALL12 |
|---|---|---|---|
| preserved across a call | a2,a3 (2) | **a2..a7 (6)** | a2..a11 (10) |
| register args | 6 | **6** | **2** — 3-arg calls un-emittable |
| first spill at depth | 11 | **6** | 4 |
| regs spilled at depth 40 | 124 | **280** | 436 |

Silicon-measured rule: `a_j` survives iff `j < 4*inc`. Save-area need: a frame entered
with `u` units requires `16*u` bytes at its top (→ 32 for CALL8). CALL12 is disqualified
by the 2-register-arg ceiling; CALL4 by 2 survivors plus a staging-vs-program-register
overlap (a6/a7). This choice *is* the allocator configuration — it writes
`caller_saved_pool_hw()` and `allocatable_pool_order()`. See
`docs/adr/2026-07-28-xtensa-abi-contract.md`.

### Argument passing and returns (matches the esp toolchain)

Verified against a real esp-toolchain-compiled fixture (`call_conv`), no divergence:
caller stores arg `i>=6` at `[caller_SP + 4*(i-6)]`; callee reads at
`[callee_SP + frame + 4*(i-6)]`. sret pointer arrives as the **first** argument.
`IsaTarget`-shaped answers: `call_arg_reg_count()=6`, `direct_ret_reg_count()=2`,
`sret_uses_buffer_for(n) = n > 2`, `lpir_call_stack_args_start(..) = 6` (or 5 when
`uses_sret && !caller_passes_ptr`) — rv32's exact formula over 6 argument registers, and
rv32's sret-swap slot logic composes unchanged (rotation is invisible to slot mapping).

### Contract + safety evidence (hardware)

- Preserved set is **exactly a2..a7** across leaf, non-leaf, and recursive callees at
  depths 1/5/8/40 — verified *through* the physical-file wrap and save-area reload path
  (depth 40 = 36 spills/36 reloads), not just shallow rotation.
- **Spill slots never collide with window save areas**: 68 dual-run cases (slot counts
  1–64 × depths 1–100 × frame padding × all three increments), each asserting
  spills == reloads, spill onset at the measured depth, and — at *address* level — that
  no handler save-area byte range ever intersects a program slot store.
- Caller-saved-spill pattern (live value in a clobbered register across a call → slot →
  back) round-trips exactly.

### Immediate legality (the `imm.rs` parameterization input)

`xt-mini-emit/src/imm.rs` is a per-opcode table (34 entries: range/scale/signedness +
the fallback lowering), verified against LLVM `.td` + AsmBackend fixups, ~90 assembler
boundary probes, and encoder round-trips. Critical facts for the port:
- **Xtensa has no `ANDI`/`ORI`/`XORI`** — an explicit `NoImmForm` entry; bitwise-immediate
  must lower to a pooled constant + register op.
- **`lp_xt_inst::encode` silently truncates** out-of-range immediates (`addi 128` →
  `-128`), so every immediate must be gated through the table.
- `l32r`'s 16-bit field is **one-extended**, not sign-extended (full reach −262144..=−4);
  a sign-extending decode turns half the range into forward offsets. This bug was found
  and fixed in *both* the emitter's assert and the emulator's executor.
- `extui` carries a joint `shift + width <= 32` constraint no per-field predicate
  expresses.
- Largest emittable frame is **32752** bytes (ENTRY caps at 32760, 16-byte rounding);
  beyond that is a documented hard error, MOVSP idiom not implemented.

All immediate rules and register-model choices are **identical on classic ESP32 (LX6)** —
verified, not merely annotated: the full corpus N-ran on classic silicon with zero
divergences (P5), and every immediate boundary agrees between the LX6 and LX7
assemblers with byte-identical encodings (P6, live test
`xt-mini-emit/tests/imm_gas_lx6.rs`; FINDINGS.md, "LX6 conformance").

## Multi-board: classic ESP32 (LX6) is a first-class target

**The deployed-hardware wedge is the classic ESP32, not the S3.** Both are now
supported and hardware-verified here; the S3 remains the convenient bring-up board.
The monorepo should inherit three shapes from this:

**1. Board is a parameter in three places, not an assumption.** This repo learned that
the hard way — "the board" was implicitly an S3 in the firmware, the emulator, and the
test harness:
- **Firmware**: `xt-runner-core` (no_std, board-agnostic: ledger, COBS/postcard
  dispatch, payload flow, watchdog policy) + per-SOC crates supplying only `Transport`,
  `CodeMem`, `PayloadWatchdog`. Reinforces the already-recorded "one fw crate per SOC
  family" decision. See `docs/adr/2026-07-28-runner-board-abstraction.md`.
- **Emulator**: `lp-xt-emu`'s memory map is a `BoardProfile`, not constants — **the
  monorepo's emulator needs the same**, because dual-run against an LX6 device with an
  S3 memory map silently compares the wrong addresses.
- **Harness**: N-run (emulator per profile + every attached board) through one code
  path, with boards **verified by the chip id they report**. Ports renumber across
  replug — the S3 moved `usbmodem1101 → 1301` mid-session when a third board appeared —
  so trusting an env var alone tests the wrong silicon.

**2. The classic code-execution model — the S3's `+0x6F0000` alias does NOT generalize.**
Measured (5 sentinels; the linear hypothesis read garbage at all five):

| Region | Write | Rule | Usable |
|---|---|---|---|
| SRAM1 (primary) | D-bus, any width | **word-mirrored**: `iram = 0x400BFFFC − (dram − 0x3FFE0000)` | ~96 KB |
| SRAM0 | its own I-bus address | identity, **32-bit aligned words only** | ~125 KB |
| RTC-fast | D-bus `0x3FF80000+` | 1:1 (`+0xC40000`) | 8 KB |
| heap (SRAM2) | — | **not executable** — no I-bus view | — |

Consequences for `lpvm-native`'s `rt_jit`: the write/exec pointer split must be a
**rule**, not an offset (`AliasRule::{Offset, Identity, WordMirrored}` is the shape that
worked), the code buffer may be a **fixed region** rather than heap-backed (so
"too large" is a real error path), and writes may need to be word-only with a
non-monotonic address walk. A trait shaped as "buffer pointer + constant alias" models
the S3 perfectly and is unimplementable on the actual target chip.

**3. LX6 vs LX7: the ISA is identical; only the memory system differs.** Verified, not
assumed — LX7-assembled golden vectors run byte-for-byte on LX6; a 171-case
dual-assembler sweep over every immediate-table boundary found zero encoding or verdict
differences; and the full emitter corpus passed on LX6 silicon with zero divergences.
**Crucially, classic ESP32 HAS hardware division** (`quos/quou/rems/remu`, div-by-zero
→ EXCCAUSE 6, `INT_MIN/−1` wraps) — the roadmap's biggest suspected divergence is
retired, so **one emitter and one ABI serve both chips**. Emitting the LX6-common subset
(wide forms; no LX7-only instructions) costs nothing and keeps it that way.

## What is NOT proven here (carry as monorepo risk)

- FPU executors (fixtures + emitter are integer-only; lightplayer's device path is
  Q32, but `fw-emu-xt` running arbitrary fork-compiled code will need FPU in the emu).
- Full `InstLog`/trace parity + cycle model (`lp-xt-emu` has hook points only).
- The `lp-emu-core` extraction itself (monorepo refactor #2 — the prize is ~2× bigger
  now that `profile/` landed in lp-riscv-emu; see readiness report).
- Classic ESP32 (LX6) FPU/timing — the ABI, immediate, and division contract is now
  silicon-verified on classic (zero corpus divergences; hardware `quos/quou/rems/remu`
  present — FINDINGS.md, "LX6 conformance") and the code-execution model is established
  (word-mirrored SRAM1 writer, C2). Still open on classic: the runner firmware's
  near-cap payload OOM (RX path transiently needs ~3× the payload, so ~33KB payloads
  panic instead of answering `PayloadTooLarge` — firmware backlog, not an ISA issue),
  and everything FPU (out of scope repo-wide).
- Absolute-symbol `CALLX8` on device (emulator-only here; PC-relative `CALL8` is the
  device-proven path; a runtime-base-discovery probe worked but is fragile — see M5).
- The MOVSP large-frame escape path (frames > 32752 bytes hard-error today).
- Real LPIR lowering, real `VInst`, and real register allocation — this repo proves the
  *contract*, not the integration.
- **Classic ESP32 under load**: RAM headroom with WiFi up (measured heap here is ~97KB
  bare-metal on a 192KB DRAM part, vs the S3's ~204KB), WS2812/RMT driving, and any
  perf/CCOUNT numbers. Classic is the tighter part — size the JIT arena against it, not
  the S3.
- **A third chip** is admitted by the design (board = parameter everywhere) but not
  exercised; the ESP32-C6 in the same drawer is RISC-V and belongs to the existing rv32
  backend, not this one.

---

# WS281x LED driver backport seam (`lp-ws281x` → lp2025)

A second, unrelated body of work in this repo (see the root `README.md`'s
"WS281x LED driver" section) that lp2025 is porting **now**, starting from
the state this document describes: `lp-ws281x` ports into `lp-fw/` unmodified,
and a new S3 backend crate implements lp2025's own existing
`Ws281xDriver`/`Ws281xOutput` registry traits over it. This section is written
so that porting agent can work from it without reading the rest of this repo.
Everything here is validated on real ESP32-S3, classic ESP32 and ESP32-C6
hardware — see `lp-ws281x/README.md`, the three `fw/led-lab-*/README.md`
files, the ADR at `docs/adr/2026-07-31-ws281x-rmt-driver-architecture.md`, and
`findings.md` in the plan directory
(`2026-esp32s3-experiment/2026-07-28-ws281x-rmt-driver/findings.md` under the
planning root) for the underlying evidence.

## What replaces what

**`lp-ws281x` replaces lp2025's `lp-fw/fw-esp32/src/output/rmt/` entirely** —
it is that driver's direct descendant, generalized and cleaned up by the same
author, not a new design. Port the crate itself unmodified (per lp2025's own
plan notes: "no S3-isms," channel count comes from the board manifest, not a
baked-in constant); the porting work is a **new backend crate** implementing
`RmtHw` for the S3's registers (see below), plus a thin adapter implementing
lp2025's `Ws281xDriver`/`Ws281xOutput` traits over an `lp_ws281x::Ws281xDriver`
instance.

**Naming collision to watch for**: lp2025's chip-agnostic seam
(`lp-core/lpc-hardware/src/drivers/ws281x/ws281x_driver.rs`) defines a *trait*
named `Ws281xDriver` (`endpoints`, `open`, supertrait `HwDriver`) plus
`Ws281xOutput` (`write(&[u8])`, `resize`) and `Ws281xConfig` (just
`byte_count`; pin comes from the endpoint address). `lp-ws281x` also exports a
type named `Ws281xDriver` — but it is a **struct**
(`Ws281xDriver<H: RmtHw, const N: usize>`, this crate's whole driver, holding
one backend and N `ChannelState`s), not a trait. The two are not the same
thing and the names will collide unqualified in the same module. The new S3
backend crate's adapter implements lp2025's `Ws281xDriver` *trait* and
`Ws281xOutput`, and internally owns (or references, if it is a `static` like
every `fw/led-lab-*` crate here) an instance of `lp_ws281x::Ws281xDriver`.

**What changed vs. the lp2025 original**, all load-bearing for the port:

- **N channels, not one.** lp2025 hardcoded channel 0 in its ISR and kept
  state in slot 0 only. `lp_ws281x::Ws281xDriver<H, N>` is const-generic over
  channel count; `on_interrupt` services every flagged channel in one pass
  (coincident causes are the steady state with several channels sharing one
  block each, not a rare accident — see the trait doc comment on `RmtHw` and
  the driver's `on_interrupt` doc).
- **A bit cursor, not an LED counter.** lp2025 assumed a ping-pong half held a
  whole number of LEDs (its 192-word window halved into exactly 4 LEDs). That
  breaks the moment a channel owns one memory block instead of four — the
  S3's 48-word block halves into 24 words = exactly one LED, which happens to
  still be whole, but do not assume it stays whole if the block plan changes;
  the classic ESP32's 64-word block halves into 32 words = 1⅓ LEDs, which is
  not. The bit-cursor core makes every half size correct on every chip
  without a special case.
- **300 µs default latch, not 50 µs.** lp2025's 50 µs latch is too short for
  modern WS2812B-V5 and WS2815 parts (WLED/NeoPixelBus's own timing tables —
  consulted as behavioral reference only, see the ADR's license posture —
  use 300 µs, and that is now this crate's `ChannelTiming::WS2812` default).
  If the lp2025 board's strips are known-older parts that need the shorter
  latch, override the timing per channel; do not silently reintroduce a
  50 µs global default.
- **Per-channel `ChannelTiming` + `ColorOrder`**, not hardcoded GRB. Configure
  each channel independently via `Ws281xDriver::configure`/
  `configure_default_clock`.
- **`BlockPlan`, shared by the driver and the backend**, not an implicit
  assumption. It decides which channels exist at all when one channel's
  window is widened to absorb the blocks above it, and it is validated (an
  overlapping plan is rejected, not silently wrong). The backend must be
  built from the *same* `BlockPlan` value the driver uses — `RmtHw::ram_words`
  remains the authority the driver trusts, but the backend's own RAM
  addressing must agree with it.
- **Warts fixed, all present in the lp2025 original and absent here**: the
  start-of-frame guard race (lp2025 planted a guard at word 0 right after
  `tx_start` and hoped the transmitter had already passed it); `static mut` +
  `transmute('static)` + `AnyPin::steal` (every `fw/led-lab-*` crate here uses
  a plain `static Ws281xDriver<Backend, N>` — `with_blocks`/`new` are `const`
  and every field is atomic, so no unsafe registry trick is needed, provided
  the new backend crate owns its pins for the program's lifetime the way a
  firmware binary can and a dynamically-reconfigured registry may not); the
  interrupt handler being re-registered on every channel construction (now
  bound once, behind an `AtomicBool`, in `install_isr`); a
  `Relaxed`-store/`Acquire`-load counter mismatch (see `state.rs`'s "Memory
  ordering" module doc for the release/acquire pairing that replaces it).

**`DisplayPipeline` stays exactly where it is.** The driver's scope is
unchanged: it takes finished `u8` RGB frames and does no color processing —
gamma, dithering, white-point, brightness all remain
`lp-core/lpc-shared`'s job, upstream of the driver, exactly as lp2025's
existing per-frame flow already has it (engine tick →
`flush_dirty_output_sinks` → `Esp32OutputProvider` → `DisplayPipeline` →
driver `write` with finished bytes). The new adapter's `Ws281xOutput::write`
implementation is a thin wrapper over `Ws281xDriver::send_blocking` (or
`send_blocking_all` if the registry starts multiple channels together) —
nothing upstream of it changes.

**Known single-channel leaks to clean up while integrating** (from lp2025's
own code, not from this repo): `fw-esp32-common/src/output/rmt_state.rs`'s
fixed `[ChannelState; 2]` with a comment noting "only [0] used" — a
per-chip RMT concern that has been living in the *common* crate; expect to
retire it, since `lp-ws281x` brings its own per-channel state. Also
`provider.rs`'s `MAX_LEDS = 256` silent truncation (duplicated in the C6
driver) and its hardcoded 60 fps double-push — neither blocks a 4-channel S3
port, both are worth flagging as separate cleanup.

## The `RmtHw` trait surface (7 methods)

Everything chip-specific a backend supplies; read `lp-ws281x/src/hw.rs` for
the authoritative doc comments (each method's contract below is a summary,
not a replacement for reading the source — the trait file is ~150 lines).
Every method takes `&self`: the driver is shared between thread context and
the interrupt handler, so a backend's state is memory-mapped registers, not
owned/mutable Rust data.

| Method | Contract | The trap |
|---|---|---|
| `ram_words(&self, ch: u8) -> usize` | Words usable by `ch` (block size × blocks assigned). **Must be stable for the driver's lifetime, at least 4, and even** — the driver splits it into two equal ping-pong halves. | An odd or sub-4 value is a `ConfigError` at `configure()`, not a panic — but it means the `BlockPlan` handed to the backend and the one handed to the driver disagreed, which should never happen if both come from one shared value (see `BlockPlan` above). |
| `write_ram(&self, ch: u8, word_idx: usize, value: u32)` | Volatile store of one RMT word into `ch`'s window, `word_idx` in `0..ram_words(ch)`. | None structural — but every backend here derives the physical address as `RAM_BASE + window_start(ch) + word_idx`, never from a per-channel base register, because none of the three chips have one. |
| `set_tx_threshold(&self, ch: u8, words: u16)` | Set `ch`'s `tx_lim`-equivalent so the next threshold event fires at the `words`'th word of the window. | **Chip-semantics trap, not a core bug, hit once already**: on the classic ESP32, the hardware register is a *repeating count of words sent*, not a window position — writing it re-arms the count from zero. The driver's alternating half/full request, unchanged, asked that register for an event every 64 words instead of every 32, and every second refill arrived a whole half late. Fixed in that backend by clamping the request to the channel's half size; **verify which semantics the new chip's register has before assuming either one.** |
| `read_pos(&self, ch: u8) -> u16` | Current hardware read pointer within `ch`'s window, **window-relative**, `0..ram_words(ch)`. | **The single biggest silent-porting trap in this trait.** The underlying register (`mem_raddr_ex` or equivalent) is an **absolute offset into the whole RMT RAM on all three chips measured so far** (S3: 10 bits into 384 words; classic: 10 bits into 512; C6: 9 bits into 192) — none of them are window-relative in hardware. Every existing backend computes `read_pos` as the raw register value **minus `window_start(ch)`**. lp2025's own C6 driver read the register raw and was *coincidentally correct*, because its one hardcoded channel's window starts at word 0 — that bug will not survive generalizing to a channel whose window does not start at 0, which is exactly what a 4-channel port does starting with channel 1. |
| `start_tx(&self, ch: u8)` | Begin transmitting `ch` from word 0 — the whole start sequence (reset the read pointer, apply pending config). Called after RAM is filled and the threshold is set. | Chip-specific self-clearing/trigger-bit semantics vary; verify on silicon rather than assuming a `modify` vs `write` pattern from another chip's backend. |
| `stop_tx(&self, ch: u8)` | Stop `ch` immediately. **Must tolerate being called on an already-idle channel** — `Ws281xDriver::abort` calls it unconditionally on every cleanup path, including ones where the channel never started. | A backend that assumes "stop" is only ever called on a running channel will misbehave on the abort-after-error and abort-after-unwind paths, which are exercised by `send_blocking`/`send_blocking_all`'s drop guards on every early return. |
| `take_interrupts(&self) -> InterruptFlags` | Read **and clear** pending causes for all channels in one shot, so no cause is lost between the read and the acknowledgement. Returns per-channel `threshold`/`end`/`error` bitsets. | The **silent-porting trap with identical PAC accessor names across three genuinely different bit layouts**: the S3 groups `INT_*` by event-then-channel (`tx_end` bits 0..=3, `tx_thr_event` bits 8..=11, …); the C6 interleaves TX and RX with **2-bit** per-channel fields at shifts that coincide with the S3's by chance, so an S3-style `0b1111` group mask silently clears *receive* causes on the C6; the classic ESP32 interleaves *by channel* (`chN_tx_end` = bit `3N`, `chN_err` = `3N+2` as a **combined TX/RX** bit — there is no separate `tx_err`). A mask or shift copied from another chip's backend compiles and appears to work until a cause it silently drops actually fires. |

## Per-chip register facts an S3 port needs (already settled, do not re-derive)

- **RAM offset**: S3 `+0x800` (`RMT_BASE + 0x800`, e.g. `0x6001_6800`).
  Classic ESP32 is also `+0x800` — numerically the same, coincidentally, not
  because the layout is shared (different block size and channel count).
  ESP32-C6 is `+0x400`, the odd one out. Getting this wrong does not fail to
  build; the transmitter silently sends whatever RAM happens to be at the
  wrong address. Every backend here proves its offset on silicon at boot with
  a two-step probe (direct store/load, then a write pushed through the
  peripheral's own FIFO port) — see any `fw/led-lab-*/src/*_rmt.rs`'s
  `probe_ram_address`, worth porting as-is.
- **`tx_thr_event` re-raise differs per chip, and it changes what
  `guard_trips` means.** The S3 re-raises an unacknowledged threshold event: a
  post-guard replacement refill can be serviced into the gap after the
  transmitter has already latched the guard word, advancing the driver's bit
  cursor by up to one half without anything reaching the wire. Consequence:
  **on the S3, `guard_trips` is a lower bound** — a truncation in a frame's
  very last half can go uncounted (never over-reported). The classic ESP32
  and the C6 do **not** re-raise, so `guard_trips` is an **exact** count on
  those two. Any telemetry or alerting built on this counter needs to know
  which chip it is reading from.
- **The classic ESP32's `CH_TX_LIM` is a repeating count, not a window
  position** (see the `set_tx_threshold` row above) — the one place a
  register semantics difference is not just a layout difference but a
  different *meaning*, and it is a real bug already found and fixed once in
  this repo's own classic backend (`fw/led-lab-esp32/src/esp32_rmt.rs`,
  `set_tx_threshold`). If lp2025's classic port reimplements this from
  scratch rather than porting the existing backend, re-read that function's
  comments before assuming the S3's semantics generalize.

## The capacity ceiling (`findings.md` §12) — constrains the 4-channel goal, but supports it

P6's stress phase found the classic ESP32's RMT refill interrupt has a hard
**throughput ceiling**, independent of WiFi load: delivered interrupt rate
flatlines at **~46,000–55,000 refills/s** no matter how much is demanded. At
the shipped `blocks_per_channel = 1` (32-word halves on that chip), a single
continuously-transmitting channel demands `800,000 / 32` = 25,000 refills/s,
so **two channels saturate the ceiling exactly** — which is why the classic
sustains only two simultaneous *equal-length* outputs today, not four, and
why that boundary is sharp rather than gradual (`findings.md` §12 has the
full `irq_hz`-vs-demand measurement). This matters to lp2025's own roadmap
(their notes: "esp32 v3" — the classic — is next after the S3), but it is a
**classic-ESP32-specific** number, not a general RMT ceiling.

**On the ESP32-S3 — lp2025's actual target for this port — the equivalent
measurement supports the 4-channel goal directly.** At `blocks_per_channel = 1`
(24-word halves, four channels, 300/256/200/150-LED strips, the same
configuration a 4-channel port would use), a 600-second stress soak measured:

| scenario | frames | truncated | lag_max/24 |
|---|---|---|---|
| idle | 220,568 | **0** | 9 |
| WiFi scan loop | 216,792 | 2,248 (**1.04 %**) | 12 |
| ESP-NOW broadcast | 74,692 | 2 (~0.003 %) | 10 |

Zero errors, zero timeouts, in every scenario (`findings.md` §11.1; also
summarized in `fw/led-lab-esp32s3/README.md`'s stress-harness section). **Say
this explicitly to whoever is deciding whether four channels is achievable on
the S3: it is, at idle and under ordinary radio use, by direct measurement,
not projection.** The only degradation is under a continuous WiFi *scan*
loop specifically (not steady-state association or ESP-NOW, which are close
to free) — a much lighter finding than the classic ESP32's, where the same
class of load truncates roughly two frames in three and is the subject of a
separate high-priority-interrupt follow-up plan (see the ADR). Nothing about
the S3 numbers blocks shipping four channels; the WiFi-scan number is a
degrade-not-break case if this port ever wants to close it further, not a
launch blocker.
