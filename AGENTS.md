# AGENTS.md — 2026-esp32s3-experiment

Feasibility + standalone-core work for lightplayer's Xtensa (ESP32-S3) backend.
Read `README.md` for what this repo is; `docs/adr/` for durable decisions.

## License discipline — HARD RULE (see docs/adr/2026-07-28-license-provenance-discipline.md)

- **NEVER copy, transliterate, or line-by-line adapt GPL source** into any crate here.
  QEMU (`oss/qemu-xtensa`), binutils/GDB (`oss/binutils-gdb`), and GCC are **behavioral
  references only** — run them, read them to understand semantics, then implement
  independently from primary specs. Do not reproduce their code.
- **Apache-2.0 LLVM material** (`oss/llvm-project-xtensa/llvm/lib/Target/Xtensa/*.td`)
  MAY be used to derive encoding *data*, IF you add a provenance header (repo/path/commit
  — template in `oss/XTENSA-REFS.md`) and the license is vendored in `licenses/`.
- Prefer primary specs (Xtensa ISA Reference Manual, ESP32-S3 TRM in `oss/xtensa-docs/`).
- If unsure whether a source is safe to copy from: **ask; do not copy.**

## Workspace layout

- **Host crates** (root virtual workspace) build on **stable** Rust: `cargo build`,
  `cargo test`. Naming: `lp-xt-*` = destined for the lp2025 monorepo backport; bare
  `xt-*` = experiment-local scaffolding.
- **Device firmware** lives under `fw/` with its own `rust-toolchain.toml` (rustup `esp`
  channel) + `.cargo/config.toml` (Xtensa target). Build by `cd fw/<crate>` — it is
  EXCLUDED from the root workspace. Xtensa asm needs `#![feature(asm_experimental_arch)]`.

## Conventions

- Golden vectors (instruction byte sequences) are **assembler-derived or
  hardware-verified only** — never hand-written from memory (spike lesson: 2/3 recalls
  were wrong). Derive from `xtensa-esp32s3-elf-objdump` output.
- N-run tests (`xt-testkit`): host tests always run the emulator on **every**
  `BoardProfile`; each attached board (per-board env vars below) also runs every
  case and all worlds must agree. Board-specific expectations live in per-board
  files (`xt-testkit/tests/board_*.rs`), never `if chip == ...` branches.
  Every unsafe block gets a `// SAFETY:`.
- Serial experiments print machine-checkable `En: PASS/FAIL/MEASURE key=value` lines.
- `scripts/capture.py <port> <seconds>` captures serial (USB-Serial-JTAG re-enumerates
  after reset; the script retries the open).

## Hardware

Two boards, named by env var and **verified by chip id** (port numbers renumber
across replug — never trust them):

- `XT_PORT_ESP32S3` — ESP32-S3 (LX7, rev v0.2, 16MB), USB-CDC. `XT_DEVICE_PORT`
  is a retained alias.
- `XT_PORT_ESP32` — classic ESP32 (LX6, rev v3.0, 4MB), USB-UART bridge @115200.

Hardware tests run **single-threaded** (`-- --test-threads=1`); a *board* is the
shared resource. An unset var skips that board; a configured-but-unreachable
board FAILS (silent skips hide regressions). A board stuck in the ROM
bootloader (aborted flash / board-info) recovers with `espflash reset --port …`.

Flash: `cd fw/xt-runner-esp32s3` (or `fw/xt-runner-esp32`) `&& cargo run --release`
(or `espflash flash --chip <esp32s3|esp32> --port <port> <elf>`).

### WS281x LED driver boards (`lp-ws281x` + `fw/led-lab-*`)

A third board joins the two above for this work: **ESP32-S3** (shared with
`xt-runner-esp32s3`), **classic ESP32** (shared with `xt-runner-esp32`), and
**ESP32-C6** (Seeed XIAO, RISC-V — this driver's origin chip, not used by the
Xtensa work above).

**Port names for all three are NOT stable — they have changed twice across
this plan's phases** (e.g. the S3's port moved mid-session when a third board
was plugged in; the classic's port string has appeared as both
`/dev/cu.usbserial-1440` and `/dev/cu.usbserial-11440` in different phases'
notes). Treat any port path recorded in a README, a plan file, or this file as
a **stale example**, never a fact to reuse unchecked. Re-enumerate instead:

1. List candidates: `ls /dev/cu.*` (macOS). USB-Serial-JTAG boards (S3, C6)
   re-enumerate after flash/reset, so a port can also disappear and
   reappear mid-session — the retry-on-open loop in `scripts/capture.py`
   exists for exactly that.
2. Confirm which candidate is which chip with `espflash board-info --port
   <candidate>` (reports chip type, not just "a device answered") — never
   assume from the path alone. The classic ESP32 is a plain USB-UART bridge
   (no chip-id handshake at the OS level the way USB-CDC/JTAG boards have),
   so this step is the only reliable check for it too.
3. Only then flash or capture against that port.

The loopback self-test (`test_loopback` feature, one per `fw/led-lab-*`
crate) is this driver's standard oracle: it proves the whole chain — pulse
encoding, ping-pong refill, guard word, per-channel timing/color-order — by
routing each TX channel's own pin into an RMT RX channel through the GPIO
matrix and asserting the decoded wire against the sent frame numerically, with
**no wires, no strips, and no external instrument**. Prefer it over reasoning
about a change from source; every phase of this plan used it (or the P6
stress harness built on the same driver) as the actual gate before calling
something done. See each `fw/led-lab-<chip>/README.md` for per-chip loopback
routing and capacity notes, and `lp-ws281x/README.md` for what it proves on
the host side.
