# Findings — ESP32-S3 dynamic-code feasibility spike

Spike run 2026-07-28 on real hardware. See [README.md](README.md) for context.

## Verdict table

| # | Experiment | Verdict | Detail |
|---|---|---|---|
| E1 | Toolchain + HAL + USB-Serial-JTAG | **PASS** | [E1](#e1--toolchain-hal-board) |
| E2 | Execute hand-assembled code from RAM | **PASS** | [E2](#e2--execute-hand-assembled-code-from-ram) |
| E3 | CALLX8 → Rust builtin + L32R literal pool | pending | |
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

Pending.

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

## What this spike did NOT test

- ws281x/RMT LED driving, serial comms protocol, radio/ESP-NOW (explicitly deferred)
- classic ESP32 (LX6) — S3 only
- performance of JIT'd code; PSRAM; real codegen from LPIR
