//! `lp-xt-emu` — a pure-Rust Xtensa (ESP32-S3 / LX7) instruction-set emulator
//! core with the windowed-register machinery.
//!
//! This crate is original code implemented from the Xtensa ISA Reference Manual
//! semantics and validated by diffing against real hardware via `xt-runner`. No
//! GPL source (QEMU, binutils) was copied or transliterated — see the repo
//! license ADR (`docs/adr/2026-07-28-license-provenance-discipline.md`) and the
//! Provenance section of this crate's README.
//!
//! ## Shape
//!
//! Mirrors `lp2025`'s `lp-riscv-emu`: a [`memory`] model, a [`cpu`] register
//! file, and one executor per instruction group (`executor/*`), so the eventual
//! monorepo backport is a merge rather than a rewrite. Decoding is delegated to
//! [`lp_xt_inst`]; this crate never re-implements it.
//!
//! ## Entry point
//!
//! [`Emulator::run`] loads a code blob into SRAM1 and invokes it exactly as the
//! device runner does — via a synthesized windowed `CALL8`, arg in `a10`
//! arriving in the callee's `a2` — returning a [`RunOutcome`].

pub mod cpu;
pub mod emu;
pub mod error;
pub mod memory;
pub mod trace;

mod executor;

pub use emu::{Emulator, RunOutcome, SyscallHandler, SyscallOutcome};
pub use error::{Trap, TrapKind};
pub use trace::{NoopTracer, TextTracer, TraceEvent, Tracer};
