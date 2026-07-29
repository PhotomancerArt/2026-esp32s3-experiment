# 2026-esp32s3-experiment

A hardware-validated **standalone Xtensa core** for [lightplayer](https://github.com/light-player)'s
Xtensa backend — plus the feasibility spike that started it. Can lightplayer's
JIT-compiled LED shaders run on Xtensa? Yes: proven end-to-end against real silicon
on **both** target chips — **ESP32-S3 (LX7)** and **classic ESP32 (LX6)**, the latter
being what most deployed WLED-class hardware actually runs.

Everything is **N-run**: the emulator executes every case on every board's memory
profile, and every attached board runs it too, through one code path. The LX6
conformance sweep found **zero ISA divergences** from LX7 — one emitter serves both.

## The core (built here, backport-ready)

Everything below is verified on a real ESP32-S3 (see [FINDINGS.md](FINDINGS.md) and
per-crate READMEs); the seam into the monorepo is [docs/BACKPORT.md](docs/BACKPORT.md).

| Crate | What | Verified |
|---|---|---|
| `lp-xt-inst` | Xtensa encode / decode / disasm | objdiff: 10,969/10,969 instructions match objdump |
| `lp-xt-emu` | Pure-Rust emulator + windowed-register machinery | matches hardware through depth-100 window recursion |
| `lp-xt-elf` | Linked-ELF loader + guest runtime | 14 Rust fixtures run on the emulator |
| `xt-mini-emit` | MiniVInst → Xtensa emitter (pools, branches, frames) | 60-case corpus agrees emu-vs-silicon on both chips |
| `xt-runner` (+core/client/proto) | On-device payload runner — send code over serial, no reflash | crash + hang recovery on S3 (USB-CDC) and classic ESP32 (UART), the hardware oracle |
| `xt-testkit` | Shared N-run harness: emulator on every board profile + every attached board | one code path serves LX7 and LX6 |

The two hardest risks of the whole project — the windowed ABI and the espup toolchain —
are retired.

## Boards

| Board | ISA | Transport | Code memory | Env var |
|---|---|---|---|---|
| ESP32-S3 rev v0.2 | LX7 | USB-Serial-JTAG | heap; execute at the `+0x6F0000` I-bus alias | `XT_PORT_ESP32S3` |
| classic ESP32 v3 | LX6 | USB-UART bridge @115200 | fixed SRAM1 region, **word-mirrored** `iram = 0x400BFFFC − (dram − 0x3FFE0000)`; the heap is *not* executable | `XT_PORT_ESP32` |

The two chips differ **only in the memory system** — the executed instruction set is
identical (LX7 golden vectors run byte-for-byte on LX6, and a 171-case dual-assembler
sweep found no encoding differences). Port numbers renumber across replug, so boards are
**verified by the chip id they report**, never by port name.

## The spike (how it started)

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

## Testing

Host tests are **N-run** (P5): the emulator runs every case on every board's
memory profile, and every attached board runs it too, through one code path
(`xt-testkit`). Boardless `cargo test --workspace` is always green; boards join
via per-board env vars, **verified against the device's reported chip id**
(port numbers are not stable across replug):

```bash
cargo test --workspace                                       # emulator-only
XT_PORT_ESP32S3=/dev/cu.usbmodemXXXX \
XT_PORT_ESP32=/dev/cu.usbserial-XXXX \
  cargo test --workspace -- --test-threads=1                 # + real silicon
```

Single-threaded with hardware — a board is a shared resource. An unset var
skips that board; a configured-but-unreachable board **fails** (silent skips
hide regressions). Board-specific expectations (capacity contracts, transport
quirks) live in `xt-testkit/tests/board_esp32s3.rs` / `board_esp32.rs`.

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
