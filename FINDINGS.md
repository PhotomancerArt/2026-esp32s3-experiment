# Findings — ESP32-S3 dynamic-code feasibility spike

Spike run 2026-07-28 on real hardware. See [README.md](README.md) for context.

## Standalone core (M0–M7) — additional hardware findings

The spike (E1–E5, below) proved feasibility. The standalone core built on it —
`lp-xt-inst`, `lp-xt-emu`, `lp-xt-elf`, `xt-mini-emit`, `xt-runner` — is all
hardware-verified; the monorepo seam is in [docs/BACKPORT.md](docs/BACKPORT.md).
New findings worth carrying:

- **The emulator matches silicon through the window machinery.** Depth-100 `call8`
  and depth-60 `call12` recursion (many WindowOverflow/Underflow spill/reload
  round-trips through emitted `ENTRY` frames) produce identical results emu-vs-device.
  Two window bugs were found by depth-bisection (overflow firing one frame late;
  per-`WindowBase` save-area table clobbered by base reuse — now driven off a
  call-stack shadow). See ADR `docs/adr/2026-07-28-emu-window-overflow-direct.md`.
- **The runner's payload load address depends on the total request-frame length**,
  not just payload length — the firmware allocates a frame-sized scratch `Vec` before
  the JIT buffer (`fw/xt-runner/src/main.rs`), so a one-byte-longer postcard `arg`
  varint moves the buffer. Absolute-address `CALLX8` on device is therefore fragile
  (a runtime-base-discovery probe works with frame-length parity but is emulator-only
  in the corpus); PC-relative `CALL8` is the robust device-proven call path.
- **Immediate legality is per-opcode and unlike RV32**: Xtensa has no `ANDI`/`ORI`/
  `XORI` (the emitter routes bitwise-immediate through a pooled constant + register
  op). `imm.rs` must become ISA-parameterized in the backport.
- **`lp-xt-emu` returns 0 on divide-by-zero; hardware raises `IntegerDivideByZero`.**
  Unexercised by the corpus (nonzero divisors only); note for the emu's trap parity.
- Real Rust fixtures compiled by the esp fork use only the supported integer subset —
  **zero unsupported-instruction traps across 14 fixtures** — so `lp-xt-inst`/`lp-xt-emu`
  coverage is sufficient for integer code. (objdump FPU-scanning is unreliable: literal
  pools at the head of `.text` disassemble as FPU garbage; the runtime trap is the gate.)

## Compiler-contract phase — additional hardware findings

Built on the standalone core: the register model + calling convention the real backend
must satisfy. Full write-up in [docs/BACKPORT.md](docs/BACKPORT.md) and ADR
`docs/adr/2026-07-28-xtensa-abi-contract.md`.

- **The windowed ABI is invisible to register allocation** — the existing lpvm-native
  allocator can be *configured* for Xtensa, not rewritten. The only structural change
  the monorepo needs is an ISA hook in `abi/frame.rs` for reserved top-of-frame bytes
  (32) where the window save areas live.
- **Register pressure is near parity with rv32**: pool of 12 vs rv32's 13, and measured
  in practice (live sets of 12 compile with zero spill slots). The rotation makes
  argument staging a separate bank, so a2–a7 are call-preserved temporaries preserved
  *for free* — rv32 pays prologue save/restore for its equivalent.
- **CALL8 chosen on measured data**: preserved a2..a7 (rule `a_j` survives iff
  `j < 4*inc`), 6 register args, first spill at depth 6. CALL12 is disqualified by a
  2-register-arg ceiling (3-arg calls cannot be emitted); CALL4 by only 2 survivors plus
  a staging/program-register overlap.
- **No spill-slot vs window-save-area collision exists** — 68 dual-run cases across slot
  counts, depths to 100, frame padding, and all three increments, with address-level
  intersection checks. This was the worst silent-corruption risk in the design.
- **Two L32R decode bugs found and fixed**: the 16-bit field is *one-extended*, not
  sign-extended. The emitter's assert refused half the legal range, and the emulator's
  executor mis-executed the far half (turning backward offsets into forward ones).
- **`lp_xt_inst::encode` silently truncates** out-of-range immediates, which is why the
  per-opcode legality table (`xt-mini-emit/src/imm.rs`) must gate every immediate. Also
  pinned: Xtensa has no `ANDI`/`ORI`/`XORI`.
- **Divide-by-zero now traps in the emulator** (EXCCAUSE 6), matching hardware on kind
  and cause for all four division ops. `INT_MIN / -1` does *not* trap — it wraps, on both.
- Argument/return convention **matches the esp toolchain exactly** (verified by
  disassembling a real compiled fixture), so no invented convention to defend later.

## Headline conclusions for the Xtensa roadmap

1. **The S3 JIT memory model is benign**: heap buffer → write via D-bus → execute at
   `+0x6F0000` I-bus alias. No PMS/memprot configuration needed bare-metal, no cache
   maintenance needed (internal SRAM is uncached). The scary parts of "JIT on Xtensa"
   are classic-ESP32 concerns, not S3 concerns.
2. **The windowed ABI behaves exactly as documented** — rotation/arg-staging
   (a10+→a2+), ENTRY/RETW interop with rustc-emitted callers, and trap-driven
   spill/reload under hand-emitted frames all verified empirically. The emitter's
   register model can be "a0–a7 preserved, a8–a15 clobbered, args at a10+."
3. **The emitter must own literal pools** (it would anyway): LLVM MC dedupes literals
   across an object, so assembler output isn't self-contained — and the L32R offset
   formula (`((PC+3) & ~3) + (imm16 << 2)`, backward-only) is hardware-verified via
   two hand-encoded instances.
4. **The abort-tier recovery posture is viable**: panic → RTC-RAM ledger → reset →
   report loop works; esp-backtrace's printed return addresses are still
   window-mangled, so the real ledger must unmangle PCs.
5. **Toolchain is workable but old-stable**: a mid-2025 espup fork builds current
   esp-hal 1.1.1 fine; Xtensa asm requires `#![feature(asm_experimental_arch)]`.
6. Encodings hand-written from memory were wrong 2 times out of 3 — every golden
   vector in the future `lp-xtensa-inst` suite must be assembler- or
   hardware-verified, never trusted from recall.

## Verdict table

| # | Experiment | Verdict | Detail |
|---|---|---|---|
| E1 | Toolchain + HAL + USB-Serial-JTAG | **PASS** | [E1](#e1--toolchain-hal-board) |
| E2 | Execute hand-assembled code from RAM | **PASS** | [E2](#e2--execute-hand-assembled-code-from-ram) |
| E3 | CALLX8 → Rust builtin + L32R literal pool | **PASS** | [E3](#e3--callx8--rust-builtin--literal-pool) |
| E4 | Window overflow/underflow under JIT frames | **PASS** | [E4](#e4--window-overflowunderflow) |
| E5 | Abort-tier recovery + measurements | **PASS** | [E5](#e5--recovery-tier--measurements) |

## Toolchain pins

| Tool | Version |
|---|---|
| espup | 0.16.0 |
| rustc (esp channel) | 1.88.0-nightly (2ab28d2e7 2025-06-24) — Espressif fork `1.88.0.0` |
| espflash | 3.3.0 |
| esp-generate (scaffold) | 0.5.0 |
| esp-hal | 1.1.1, features `esp32s3`, `log-04`, `unstable` |
| esp-alloc | 0.10.0 (`internal-heap-stats`) |
| esp-backtrace | 0.19.0 (`esp32s3`, `panic-handler`, `println`) |
| esp-println | 0.17.0 (`esp32s3`, `jtag-serial`, `log-04`) |

Notes:
- The esp toolchain on this machine predates esp-hal 1.1.1 by ~11 months and builds it
  fine — the fork's MSRV story is currently unproblematic for this cohort.
- Xtensa requires the Espressif rustc fork (no upstream target); the repo pins
  `channel = "esp"` in rust-toolchain.toml, quarantined from lp2025's pinned nightly.
- Crate is named `esp32s3-experiment` (repo keeps the year prefix): rustc crate names
  cannot start with a digit; esp-generate happily templated the invalid name.
- esp-generate 0.5.0 templated esp-hal `=1.0.0-rc.0`; bumped to 1.1.1 with the companion
  cohort lp2025 uses for C6 (esp-backtrace 0.19 dropped the old `exception-handler`
  feature — removed).

## Board identity (the desk unit)

`espflash board-info`:

```text
Chip type:         esp32s3 (revision v0.2)
Crystal frequency: 40 MHz
Flash size:        16MB
Features:          WiFi, BLE
MAC address:       d8:3b:da:75:c9:c4
```

Richer than the project's floor pin (**4MB flash / no PSRAM / 512KB SRAM**). PSRAM not
probed and not enabled — nothing in this spike may depend on it.

## E1 — toolchain, HAL, board

**PASS.** Serial evidence:

```text
E1: PASS esp_hal=1.1.1 heap_free=204800
spike: idle heap_free=204800
```

- Build: `cargo build --release` clean, ~53s cold.
- Flash: app 90,848 bytes into a 1MB partition (default partition table), `espflash flash`.
- Logging over USB-Serial-JTAG (`esp-println` `jtag-serial`) works; the port re-enumerates
  after reset (~1–2s) — `scripts/capture.py` retries open for this reason.
- 200KB esp-alloc heap configured; `HEAP.free()` reports as expected.

## E2 — execute hand-assembled code from RAM

**PASS.** Dynamically generated code in a heap buffer executes on ESP32-S3 with no
special configuration. Serial evidence:

```text
E2A: PASS value=42
E2: PASS value=42 write_addr=0x3fc8a25c exec_addr=0x4037a25c barriers=memw+isync
E2C: PASS value=42 barriers=none
```

Findings:

- **The SRAM1 dual-mapping works exactly as the memory map says**: write via the D-bus
  address, execute at `D + 0x6F_0000` (I-bus alias). Constants and range asserts in
  [src/jitbuf.rs](src/jitbuf.rs); source: esp-hal `ld/esp32s3/memory.x` + S3 TRM.
  The esp-alloc heap (a static in `dram_seg`) lands in SRAM1, so plain heap allocations
  are JIT-usable.
- **No PMS/memprot obstacle** in bare-metal esp-hal (default `esp_hal::init`) with the
  esp-idf-compat bootloader. Nothing had to be configured or disabled. (ESP-IDF's
  software memprot is an IDF feature; it simply isn't armed here.)
- **No cache maintenance required**: E2C executes freshly written bytes with *no*
  barriers. Internal SRAM is uncached on S3 (cache fronts external flash/PSRAM only).
  Recommendation for the real emitter: keep one `memw + isync` after emission anyway —
  cost is nil and it guards buffer-reuse/prefetch edge cases this probe doesn't cover.
- **E2D (identity-execution probe)**: jumping to the D-bus address faults as expected —
  `InstrError`, `EXCCAUSE=2` (InstructionFetchError), `EXCVADDR=0x3FC8A274` (the exact
  D-bus address). esp-hal's exception handler prints a full context dump (all ARs, SAR,
  EXCCAUSE/EXCVADDR, LBEG/LEND/LCOUNT, FP regs) — good raw material for the future
  fw-side fault reporting.
- **Return-address mangling is visible in real backtraces**: the E2D backtrace's last
  frame prints `0x7fc8a271` — a windowed return address with the top-2-bit window
  increment still embedded (raw value, un-unmangled by esp-backtrace 0.19). The blame
  ledger must unmangle (`addr & 0x3FFF_FFFF | region_bits`) before recording PCs.
- **Lesson reconfirmed**: of 3 hand-written encodings from memory, 2 were wrong
  (assembler chose wide `movi`/`retw` forms). All golden vectors are objdump-derived,
  per plan.

## E3 — CALLX8 → Rust builtin + literal pool

**PASS.** Serial evidence:

```text
E3A: PASS result=126
E3: PASS result=126 builtin_addr=0x42016d10 lit_addr=0x3fc8a27c
```

Findings:

- **The pool-before-code layout works and transfers verbatim**: the reference blob
  (literal `.word` at +0, code from +4) and the RAM copy share identical relative
  layout, so the assembler's encoded L32R offset (`81 fe ff`, imm16 = −2 words) runs
  unchanged from the heap buffer. Runtime patching of the literal slot with the live
  `spike_builtin` address is the JIT's first "relocation" — trivial.
- **Argument staging across the rotation confirmed**: caller `a10` → callee `a2` (after
  callee's ENTRY), return value back in caller `a10`. The emitter's model — "a0–a7
  preserved, a8–a15 clobbered by a CALLX8, args at a10+" — behaved exactly as documented.
- **RAM-resident code calling flash-resident code** (0x4037… → 0x4201…) crosses memory
  regions with no issue.
- L32R range rule verified concretely: target = `(PC + 3 + (imm16 << 2)) & ~3`,
  backward-only — pool slots must be ≤ ~256KB behind the referencing instruction and
  4-byte aligned; pool-at-buffer-start satisfies both for any sane buffer size.
- Assembler chose wide forms again (`mov a2, a10` emitted as wide `or a2, a10, a10`,
  3 bytes) — narrow `.n` forms are an optimization the future emitter can ignore
  initially (density is optional per-instruction).

## E4 — window overflow/underflow

**PASS.** Serial evidence:

```text
E4A: PASS depth=100 result=100 mixed=121 sp=0x3fcdb5f0
E4: PASS depth=100 result=100 mixed=121
```

Findings:

- **JIT-emitted frames are first-class citizens of the window machinery.** 100 levels of
  self-recursion through hand-emitted `ENTRY` frames (≫ the ~8 frames the 64-register
  file holds) forces many WindowOverflow/Underflow round-trips; `f(depth) == depth`
  arithmetic — and the mixed Rust → JIT×100 → Rust-builtin chain (`121`) — only survive
  if every spill/reload of our frames' save areas was correct. Both PASS, first flash.
- **Vector provenance**: `_WindowOverflow4/8/12` / `_WindowUnderflow*` come from
  `xtensa-lx-rt 0.22.0` (`src/exception/asm.rs`, pulled in by esp-hal), placed in
  `vectors_seg` at `0x40378000`. Nothing to install manually.
- **Frame sizes**: `entry a1, 32` (E4) and `entry a1, 48` (E3) both exercised — the
  ABI's 16-byte base-save-area minimum is comfortably covered by either.
- Stack: SP ≈ `0x3fcdb5f0` (top of dram_seg); 100 frames × 32B ≈ 3.2KB used — trivial.
- **Assembler literal management discovery (design-relevant)**: LLVM MC *deduplicates
  literals across the object* — the reference blob B's `.word spike_builtin` was elided
  and its `l32r` rewritten to reference E3's pool, outside the blob. Assembler-produced
  code is therefore NOT guaranteed self-contained. Consequence for the roadmap: the JIT
  emitter must own literal-pool layout and encode `l32r` offsets itself (it would
  anyway); golden vectors for pooled code must be constructed, not copied. The two
  re-encoded L32Rs in GV3b (computed by hand with the `((PC+3) & !3) + (imm16 << 2)`
  formula) executing correctly on hardware is itself evidence the formula understanding
  is right.

## E5 — recovery tier + measurements

**PASS** (ledger round-trip) + measurements recorded. Serial evidence (full cycle):

```text
E5: boot 1 of build 1785267618; will panic intentionally after measurements
E5: MEASURE heap_free=204800
E5: MEASURE arena_64k=ok heap_free_after=139264
E5: MEASURE largest_block~=204000
PANIC (rebooting, blame recorded): panicked at src/e5.rs:78:5:
E5 intentional panic (code 0xdead0001)
rst:0x3 (RTC_SW_SYS_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)
E5: PASS ledger_survived=true boot_count=2 prev_code=0xdead0001 prev_line=78
```

Findings:

- **The abort-tier recovery loop works end-to-end**: custom `#[panic_handler]` records
  blame into RTC fast RAM (`#[esp_hal::ram(unstable(rtc_fast, persistent))]` statics —
  note the syntax; `Persistable` is implemented for `portable_atomic` types so the
  ledger is safe atomics, no `static mut`), prints, `esp_hal::system::software_reset()`;
  next boot reads and reports the prior panic. Replaces esp-backtrace's handler (dep
  dropped) — backtrace capture is future work for the real blame ledger.
- **RTC RAM survives reflashing too** (power stays up), so the ledger carries a
  build-id (unix-time injected by build.rs) to distinguish fresh-flash from
  post-panic reboot. It also survived an external `espflash reset` (boot_count kept
  incrementing). Power-cycle behavior (expected: contents lost) not exercised —
  the board stayed USB-powered throughout.
- **Heap numbers** (200KB configured heap, minimal firmware): free=204800 at boot;
  64KB JIT-arena alloc drops it by exactly 65536; largest single block ≈ 204000.
  Nothing else contends for DRAM in this skeleton — the real fw budget question
  remains open (and is chiefly a *classic*-ESP32 concern).
- **Flash size**: 95,632 bytes app image (minimal no_std + esp-hal + println + alloc).
- **Measurement gotcha worth remembering**: with fat LTO, unused alloc/dealloc pairs
  are *elided* — the first numbers showed a largest-block larger than the heap.
  `black_box` + a volatile write through the pointer makes the probe real.
- esp-hal `unstable` feature: required for `system::software_reset` and the `ram`
  macro options used here (already enabled since P1).

## Golden vectors

Collected here as they are produced; seed tests for `lp-xtensa-inst` encode/decode and
emulator conformance. All derived from `xtensa-esp32s3-elf-objdump` of toolchain-assembled
references, never hand-written.

### GV1 — `spike_stub42` (minimal windowed function)

```text
36 41 00    entry  a1, 32     ; word 0x004136
22 a0 2a    movi   a2, 42     ; word 0x2aa022 (wide form)
90 00 00    retw              ; word 0x000090 (wide form)
```

### GV2 — `spike_call_blob` (literal pool + CALLX8 builtin call)

```text
+0  xx xx xx xx  .word spike_builtin   ; literal slot, runtime-patched
+4  36 61 00     entry  a1, 48         ; word 0x006136 (imm12 = 48>>3 = 6)
+7  81 fe ff     l32r   a8, <-8>       ; word 0xfffe81 (imm16 = -2 words → slot at +0)
+10 a2 a0 2a     movi   a10, 42        ; word 0x2aa0a2
+13 e0 08 00     callx8 a8             ; word 0x0008e0
+16 a0 2a 20     mov    a2, a10        ; word 0x202aa0 (wide or a2, a10, a10)
+19 90 00 00     retw                  ; word 0x000090
```

Entry point at +4. L32R target formula: `((PC + 3) & ~3) + (imm16 << 2)`.

### GV3a — `spike_rec` (self-recursive windowed stub, `f(d) = d`)

See `REC_BLOB_BYTES` in [src/e4.rs](src/e4.rs) — verbatim objdump copy (self literal
at +0, code at +4; `beqz` is blob-internal so it survives copying).

### GV3b — `spike_recb` (mixed: recursion + builtin base case, `f(d) = d + 21`)

See `RECB_BLOB_BYTES` in [src/e4.rs](src/e4.rs) — constructed, not copied: two-slot
pool (+0 self, +4 builtin), both L32Rs re-encoded by hand (imm16 = −4 and −7) because
LLVM MC deduped the reference's second literal out of the blob.

## What this spike did NOT test

- ws281x/RMT LED driving, serial comms protocol, radio/ESP-NOW (explicitly deferred)
- classic ESP32 (LX6) — S3 only (since covered: see the classic C1–C5 section below)
- performance of JIT'd code; PSRAM; real codegen from LPIR

---

# Findings — classic ESP32 (LX6) experiment ladder (C1–C5)

Run 2026-07-28 on real hardware (`fw/spike-esp32`), mirroring the S3 spike's
E1–E5 for the classic chip. Central question: how does classic ESP32 execute
dynamically written code, given that S3's "the heap is executable" does not
hold. Answered empirically below.

## Verdict table

| # | Experiment | Verdict | Detail |
|---|---|---|---|
| C1 | Toolchain + HAL + UART bridge | **PASS** | [C1](#c1--toolchain-hal-uart) |
| C2 | Code-execution model discovery | **PASS** | [C2](#c2--the-classic-code-execution-model) |
| C3 | CALLX8 + L32R pool builtin call (GV2) | **PASS** | [C3](#c3--windowed-abi-on-lx6) |
| C4 | Window overflow/underflow depth 100 (GV3a/b) | **PASS** | [C4](#c4--window-machinery-on-lx6) |
| C5 | Abort-tier recovery + measurements | **PASS** | [C5](#c5--recovery-tier--measurements) |

**Headline: all three candidate regions execute dynamically written code, the
LX7-assembled golden vectors ran byte-for-byte unmodified on LX6, and the only
new constraints are (a) SRAM1's word-*mirrored* dual mapping and (b) word-only
data access to I-bus addresses.**

## Board identity (the classic desk unit)

`espflash board-info`:

```text
Chip type:         esp32 (revision v3.0)
Crystal frequency: 40 MHz
Flash size:        4MB
Features:          WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
MAC address:       94:b5:55:c8:c8:c4
```

Matches the project floor pin (4MB flash, no PSRAM). Port
`/dev/cu.usbserial-1440` — a **USB-UART bridge**, no native USB.

## C1 — toolchain, HAL, UART

**PASS.** `C1: PASS esp_hal=1.1.1 chip=esp32 heap_free=98304`. Same espup
toolchain and crate cohort as the S3 spike (esp-hal 1.1.1 with `esp32`
feature, esp-bootloader-esp-idf/esp32, esp-alloc 0.10.0, esp-println 0.17.0
with `esp32` + `uart`); the only per-chip changes are feature names, the
`xtensa-esp32-none-elf` target, and `espflash --chip esp32`.

Two UART findings that cost real debugging time:

- **esp-println's `uart` feature writes the UART0 FIFO and never programs the
  baud divisor.** The ROM leaves a divisor for its own clock tree; after
  `esp_hal::init()` reclocks (CpuClock::max), output becomes garbage at every
  standard baud. Fix: construct `esp_hal::uart::UartTx` (115200 8N1, GPIO1)
  once at boot and keep it alive; esp-println's raw FIFO writes then go out at
  115200. The S3 never saw this because USB-Serial-JTAG has no baud.
- `scripts/capture.py` gained an optional third arg (baud): a real bridge
  keeps whatever speed the previous opener left, so captures must pin 115200.

## C2 — the classic code-execution model

**PASS — this is the deliverable.** Probes: read-backs first (distinct
sentinels, multiple offsets), then GV1 execution per region, plus two
sacrificial fault probes. Serial evidence (condensed; full lines in the run
log format `C2a/b/c/x/n/f/g`):

```text
C2a: PASS rtc_mapping=1to1
C2b: MEASURE off=0x0 want=0xc0de0000 h1@0x400b0000=0x58503a2b h2@0x400afffc=0xc0de0000
C2b: MEASURE off=0x100 want=0xc0de0100 h1@0x400b0100=0x80511dad h2@0x400afefc=0xc0de0100
C2b: PASS sram1_rule=word_mirrored iram=0x400BFFFC-(dram-0x3FFE0000)
C2c: PASS word_write=ok got0=0xf00dface got1=0x1badb002
C2x: PASS region=rtc_fast value=42 exec_addr=0x400c1900
C2x: PASS region=sram0 value=42 exec_addr=0x4009c100
C2x: PASS region=sram1_word_mirrored value=42 exec_addr=0x400b0800
C2n: PASS region=sram1_word_mirrored value=42 exec_addr=0x400b0900  (barriers=none)
```

The model, in precise terms:

| Region | Write via | Execute at | Address rule | Usable (this fw) |
|---|---|---|---|---|
| **SRAM1** | D-bus `0x3FFE_0000..0x4000_0000` (any width) | I-bus `0x400A_0000..0x400C_0000` | **word-mirrored**: `iram = 0x400B_FFFC − (dram − 0x3FFE_0000)` | ~96KB free (dram2_seg `0x3FFE_7E30..0x3FFF_FF80`; lower SRAM1 is ROM-data/stack reservations, partially reclaimable) |
| **SRAM0** | its own I-bus address, **32-bit aligned words only** | same address (identity) | `iram = iram` — no D-bus view exists | ~125KB (128KB iram_seg minus ~5.4KB of vectors + `.rwtext`) |
| **RTC fast** | D-bus `0x3FF8_0000 + off` (any width) | I-bus `0x400C_0000 + off` | clean 1:1, `iram = dram + 0xC4_0000` | 8KB minus the ledger (PRO_CPU only) |
| SRAM2 (dram_seg, the heap) | D-bus | — | **not executable** — no I-bus view | n/a for code |

- **SRAM1 is genuinely word-mirrored** — H1 (linear) read garbage at all 5
  probe offsets, H2 matched all 5, so the two windows run in opposite
  directions at word granularity. Consequence for code layout: writing
  I-contiguous code means walking the D-bus **downward** word by word. The
  spike's `CodeSpot` writer keys everything on the I-bus layout (word `i` of
  code at `iram_base + 4i`) and computes the write address per word, which
  absorbs the mirroring in one line of address math. Bytes within each 32-bit
  word are NOT swapped (the little-endian words are verbatim).
- **Word-only access to I-bus addresses**: aligned 32-bit stores/loads to
  SRAM0 work; a byte store faults —
  `C2f` → `LoadStoreError, EXCCAUSE=3, EXCVADDR=0x4009C001` (full context
  dump captured). Emit code as word-aligned u32 volatile writes, zero-padded.
- **D-bus addresses are not fetchable**: `C2g` executing GV1 at its D-bus
  address `0x3FFF_0400` → `InstrError, EXCCAUSE=2, PC=EXCVADDR=0x3FFF0400` —
  exact mirror of the S3's E2D.
- **No barriers required** (C2n: fresh code executed with no `memw`/`isync`,
  internal SRAM uncached, same as S3) — keep the belt-and-suspenders
  `memw + isync` after emission anyway, cost is nil.
- No PMS/memprot obstacle bare-metal, same as S3: nothing configured, all
  three regions fetch freshly written RAM.

## C3 — windowed ABI on LX6

**PASS.** `C3A: PASS result=126` (toolchain-assembled flash reference), then
`C3: PASS result=126 region=sram1_word_mirrored builtin_addr=0x400d9998` —
GV2 (pool-before-code, runtime-patched literal, `l32r` imm16=−2, CALLX8 into
a windowed Rust builtin, arg staging a10→a2 and return a2→a10) ran **byte-for-
byte unmodified** from mirrored-SRAM1. RAM-resident code calling
flash-resident code (0x400B… → 0x400D…) crosses regions with no issue.

## C4 — window machinery on LX6

**PASS.** `C4A: PASS depth=100 result=100 sp=0x3ffdfe90`, then from
mirrored-SRAM1: `C4: PASS depth=100 result=100 mixed=121` with a depth sweep
1..32 all-correct (first spill ~depth 6 per the CALL8 model; the spill itself
is architecturally invisible — correctness at every depth is the observable).
Same `_WindowOverflow/Underflow` vectors as S3 (xtensa-lx-rt 0.22.0 via
esp-hal). JIT-emitted ENTRY frames are first-class on LX6 exactly as on LX7.

## C5 — recovery tier + measurements

**PASS.** Full cycle in one capture:

```text
C5: MEASURE heap_free=98304
C5: MEASURE arena_64k=ok heap_free_after=32768
C5: MEASURE largest_block~=97600
PANIC (rebooting, blame recorded): panicked at src/c5.rs:85:5:
C5 intentional panic (code 0xdead0001)
rst:0x3 (SW_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
C5: PASS ledger_survived=true boot_count=2 prev_code=0xdead0001 prev_line=85
```

- `#[esp_hal::ram(unstable(rtc_fast, persistent))]` **works unchanged on
  classic** (RTC fast here is 8KB at DRAM `0x3FF8_0000` / IRAM `0x400C_0000`,
  PRO_CPU only); the ledger survives software resets and reflashes.
- Heap: 96KB configured (classic dram_seg is only 192KB — SRAM2 — vs S3's
  345KB); free=98304 at boot, 64KB arena drops it by exactly 65536, largest
  block ≈ 97600. The JIT arena itself will NOT come from this heap on classic
  (not executable) — code goes to SRAM1/SRAM0 directly.
- Flash image: 105,104 bytes into a 4MB-table factory partition (3,932,160 B).
- **UART bridge reset behavior (P3 cares)**: the serial port does **not**
  drop across device resets — a single open capture spanned panic → SW reset
  → boot 2. But *opening* the port can itself reset the board (DTR/RTS
  auto-reset wiring), so deterministic full-boot captures should use
  `espflash reset` with the port already open, and hosts must not treat
  "port stayed open" as "device never rebooted".
- **New gotcha**: `software_reset()` immediately after a println truncates
  the message — the UART TX FIFO doesn't drain (USB-CDC on S3 was immune).
  The panic handler must wait (~300ms covers a full exception dump at
  115200) before resetting, or the fault report is lost.

## LX6 vs LX7 divergences observed

**None in the executed instruction set.** Specifically:

- GV1/GV2/GV3a/GV3b (assembled for LX7 on the S3 spike) ran **unmodified** on
  LX6, and re-assembling their shapes with `xtensa-esp32-elf-as` (LX6 GCC
  assembler) produces byte-identical encodings for every wide-form
  instruction (`entry`, wide `movi`, `l32r`, `callx8`, wide `or`/`mov`, wide
  `addi`, `beqz`, `retw`). One cosmetic toolchain difference: GNU as defaults
  to narrow `.n` forms where LLVM MC chose wide forms — both encodings are
  legal on both cores (density option present on each).
- The windowed ABI (rotation, arg staging, spill/reload) behaved identically.
- The divergence is all in the **memory system**, not the core: S3 =
  uniform `+0x6F_0000` dual mapping, heap executable; classic = per-block
  buses, word-mirrored SRAM1, word-only I-bus data access, heap NOT
  executable.

## Consequence for the multi-board runner (P2+)

A payload runner comparable to the S3's is supported: ~96KB of
mirrored-SRAM1 (more if ROM reservations are reclaimed) or ~125KB of SRAM0
dwarf the 8KB RTC-fast ceiling. The runner's code memory abstraction must be
an I-bus-keyed word writer (the `CodeSpot` shape in
`fw/spike-esp32/src/codemem.rs`) rather than S3's "alias offset on a heap
pointer" — that is exactly the per-SOC trait boundary P2 plans for.

# Findings — LX6 conformance (P6)

Adjudicated 2026-07-28 against the classic ESP32 v3, building on the P5 N-run
harness. Verdict: **every "LX6-identical" annotation held — zero divergences,
none corrected.** The annotations in `xt-mini-emit/src/{gpr,abi,imm}.rs`, the
ABI-contract ADR, `docs/BACKPORT.md`, and the xt-mini-emit README are upgraded
from asserted (ISA-derived) to verified, each citing the evidence below.

## Verified on classic silicon (P5 N-run corpus — zero divergences)

The accumulated dual-run corpus ran emulator-vs-classic-board through
`xt-testkit` with no disagreement in any world:

- **Instruction/emitter corpus**: arithmetic, pooled constants (`l32r`),
  loops, branches in both directions, the relaxed long branch (inverted
  `beqz` over `j`), Select, the full icmp matrix, slots, `call8`, recursion
  to depth 100, FuelCheck.
- **Register/ABI contract**: the call-boundary torture — the preserved set is
  *exactly* `a2..a7` on LX6 too, including through the spill/reload path at
  depth (the `a_j` survives iff `j < 8` rule).
- **Window machinery**: spill/reload to depth 100; no spill-slot vs
  save-area collision (`FRAME_TOP_RESERVED_BYTES = 32` holds); C4's depth
  sweep matched the CALL8 model (first spill ~depth 6).
- **Stack args / sret**: the P4 conventions (7th+ args on the stack, sret
  pointer, 2-word direct returns) behave identically.
- **Division — the roadmap's biggest suspected divergence, retired**: LX6
  **has** the hardware divider (`quos/quou/rems/remu`; also confirmed present
  in the LX6 assembler's core config). Div-by-zero traps `EXCCAUSE=6` and
  `INT_MIN / -1` wraps without trapping — both exactly as on LX7. No
  software-division path is needed for classic ESP32.

## Immediate legality: dual-assembler boundary probe (P6's new evidence)

The `imm.rs` table (34 entries) had been probed only against the S3/LX7
assembler. P6 re-probed **every boundary with both** `xtensa-esp32-elf-as`
(LX6) and `xtensa-esp32s3-elf-as` (LX7), `--no-transform`, binutils 2.43.1
(crosstool-NG esp-14.2.0_20240906): **171 cases — zero verdict
disagreements, zero encoding differences, zero deviations from the table.**
Now a live test, `xt-mini-emit/tests/imm_gas_lx6.rs` (skips loudly when the
espup toolchain is absent; `XT_XTENSA_GAS_DIR` overrides the location).
Load-bearing checks, all identical on both cores:

- `addi`/`addmi`/`movi` (+ density `addi.n`/`movi.n`, incl. `addi.n`'s
  excluded 0) at min/max/one-beyond; load/store offset scaling for
  `l8ui`/`l16ui`/`l16si`/`l32i`/`s8i`/`s16i`/`s32i` and the `.n` forms.
- Branch reach: RRI8 ±128, BRI12 ±2048, `beqz.n`/`bnez.n` forward-only 0..63,
  `j` ±128KB — probed at the exact displacement with layout padding, both
  directions.
- `call0/4/8/12`: ±512KB word-scaled reach and target 4-alignment.
- `l32r`: backward-only, **one-extended** field — the far half (field 0x7FFF
  = −131076) and the full −262144 reach assemble identically; forward and
  beyond-reach rejected identically.
- `entry`'s frame field (0..32760, multiple of 8); `slli`/`srli`/`srai`/
  `ssai`/`sext`/`bbci`; `extui` incl. the joint `shift + width <= 32` rule.
- **`andi`/`ori`/`xori` do not exist on either core** ("unknown opcode" from
  both assemblers) — the `NoImmForm` entries hold on LX6.
- `b4const`/`b4constu` membership (incl. 32768/65536 legal, 0/1/32767/65535
  not).
- gas's two `slli` quirks are the same on both cores: sa=32 accepted by gas
  (the table follows LLVM and keeps it illegal — a deliberate conservative
  subset), sa=0 rejected under `--no-transform`.

## Remains unverified on LX6 (stated, not implied away)

- **FPU** — out of scope repo-wide (integer-only corpus, fixtures, emitter).
- **Cycle counts / perf model** — out of scope for this repo.
- Silicon execution covers the corpus's instruction subset; `imm.rs` entries
  the emitter does not currently emit (e.g. `bbci`, or `sext` at bit
  positions other than the emitted 7) are assembler- and encoder-verified on
  both cores but not silicon-executed.
- Absolute-symbol `CALLX8` on device — still emulator-only on **both** chips
  (M5 finding, unchanged by this phase).
- The classic runner firmware's near-cap payload OOM (~33KB payloads panic
  instead of answering `PayloadTooLarge`) is a known **firmware** backlog bug
  (RX path transiently needs ~3× the payload), not an ISA divergence; no
  corpus case comes near it (largest ~2.7KB), so no case was skipped for
  capacity.
