# Findings — ESP32-S3 dynamic-code feasibility spike

Spike run 2026-07-28 on real hardware. See [README.md](README.md) for context.

## Verdict table

| # | Experiment | Verdict | Detail |
|---|---|---|---|
| E1 | Toolchain + HAL + USB-Serial-JTAG | **PASS** | [E1](#e1--toolchain-hal-board) |
| E2 | Execute hand-assembled code from RAM | **PASS** | [E2](#e2--execute-hand-assembled-code-from-ram) |
| E3 | CALLX8 → Rust builtin + L32R literal pool | **PASS** | [E3](#e3--callx8--rust-builtin--literal-pool) |
| E4 | Window overflow/underflow under JIT frames | pending | |
| E5 | Abort-tier recovery + measurements | pending | |

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

Pending.

## E5 — recovery tier + measurements

Pending.

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

Entry point at +4. L32R target formula: `(PC + 3 + (imm16 << 2)) & ~3`.

## What this spike did NOT test

- ws281x/RMT LED driving, serial comms protocol, radio/ESP-NOW (explicitly deferred)
- classic ESP32 (LX6) — S3 only
- performance of JIT'd code; PSRAM; real codegen from LPIR
