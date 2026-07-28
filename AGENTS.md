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
- Dual-run tests: host tests always run the emulator; when `XT_DEVICE_PORT` is set they
  also drive the S3 via `xt-runner-client` and diff. Every unsafe block gets a `// SAFETY:`.
- Serial experiments print machine-checkable `En: PASS/FAIL/MEASURE key=value` lines.
- `scripts/capture.py <port> <seconds>` captures serial (USB-Serial-JTAG re-enumerates
  after reset; the script retries the open).

## Hardware

ESP32-S3 (rev v0.2, 16MB flash) on USB. Flash: `cd fw/<crate> && cargo run --release`
(or `espflash flash --chip esp32s3 --port <port> <elf>`).
