# 2026-esp32s3-experiment

A hardware feasibility spike for [lightplayer](https://github.com/light-player)'s planned
**Xtensa backend**: can lightplayer's JIT-compiled LED shaders run on ESP32-S3?

## Why

lightplayer compiles shaders to native machine code on-device. Today that backend
targets RISC-V (ESP32-C6). But the vast majority of deployed WLED-class LED hardware —
the controllers already screwed to people's houses — runs on **Xtensa** chips (classic
ESP32 and ESP32-S3). Supporting that hardware means building an Xtensa code generator,
emulator, and firmware image.

Before committing to that roadmap, this repo answers the questions that could kill or
reshape it, on real hardware, in a few days:

| # | Experiment | Question |
|---|---|---|
| E1 | Toolchain hello | Does the espup forked-Rust + esp-hal + espflash loop work? |
| E2 | RAM execution | Can hand-assembled windowed code in a heap buffer be called via a casted fn pointer? (memory protection, I/D-bus aliasing, cache sync) |
| E3 | Builtin boundary | Can JIT code `CALLX8` back into Rust functions, with an `L32R` literal pool for address materialization? |
| E4 | Window traps | Do WindowOverflow/Underflow spills work under JIT-emitted stack frames? (recursion depth 100 through a 64-register file) |
| E5 | Recovery + numbers | Is a `panic=abort` + RTC-RAM blame-ledger tier viable? What are the heap/flash numbers? |

Results live in **[FINDINGS.md](FINDINGS.md)**, including every hand-assembled byte
sequence as a golden vector for the future `lp-xtensa-inst` / emulator test suites.

## Layout

Two toolchains, split by directory (see [AGENTS.md](AGENTS.md)):

- **Host crates** — root virtual workspace, **stable** Rust (`cargo build` / `cargo
  test`). `lp-xt-*` crates are destined for the lightplayer monorepo; bare `xt-*` crates
  are experiment-local scaffolding.
- **Device firmware** — under `fw/` (e.g. `fw/spike`, the original feasibility firmware),
  each with its own `rust-toolchain.toml` pinning the Espressif `esp` channel. Excluded
  from the root workspace; build by `cd`-ing in.

License discipline (no GPL source; Apache LLVM derivation with provenance) is
non-negotiable — see [the ADR](docs/adr/2026-07-28-license-provenance-discipline.md).

## Running

Requires [espup](https://github.com/esp-rs/espup) (installs the Xtensa Rust fork — there
is no upstream rustc target for Xtensa) and espflash, with an ESP32-S3 on USB:

```bash
cargo install espup espflash --locked
espup install
cd fw/spike && cargo run --release   # builds, flashes, opens the monitor
```

Every experiment prints machine-checkable lines: `En: PASS key=value ...`,
`En: FAIL reason=...`, `En: MEASURE key=value`.

## Status

**Spike complete — all five experiments PASS on hardware** (ESP32-S3 rev v0.2,
2026-07-28). Headline: dynamically generated windowed code in a heap buffer executes
via the SRAM1 I-bus alias with no memory-protection obstacles and no cache maintenance;
CALLX8 into Rust builtins with runtime-patched literal pools works; window
overflow/underflow spilling is correct under hand-emitted frames at recursion depth
100; and a panic=abort + RTC-RAM blame-ledger recovery tier round-trips a real panic.
See [FINDINGS.md](FINDINGS.md) for details, golden vectors, and gotchas.

This is proof-of-concept code around the compiler/memory-model questions only — no LED
driving, no comms protocol, no radio.
