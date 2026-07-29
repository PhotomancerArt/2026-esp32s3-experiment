//! xt-runner-core — the board-agnostic core of the xt-runner payload firmware.
//!
//! The runner is the *hardware oracle* for the standalone Xtensa core: a host
//! sends a `LoadExec` payload (see `xt-runner-proto`), the runner copies it into
//! executable memory, calls it, and replies with the result — or, if the payload
//! faults or hangs, a structured `CrashReport` delivered on the next boot after
//! an auto-reset.
//!
//! Everything that does not depend on a specific ESP32 variant lives here:
//!
//! - the RTC crash ledger *logic* ([`ledger`]) — arm/disarm/record/boot-report
//!   over storage the firmware places in persistent RTC RAM,
//! - COBS frame accumulation and postcard dispatch of `Request` → `Response`
//!   ([`runner`]),
//! - the payload flow (load → arm → sync → call → disarm) and the watchdog
//!   policy around it.
//!
//! Board-specific behavior enters through three traits the firmware implements:
//! [`Transport`] (the serial channel), [`CodeMem`] (how dynamically written
//! code becomes executable — radically different between S3 and classic, see
//! the trait docs), and [`PayloadWatchdog`] (the hang-recovery reset).
//!
//! `no_std + alloc`, builds on stable for the host so the ledger and framing
//! logic are unit-testable off-device.

#![no_std]

extern crate alloc;

#[cfg(test)]
extern crate std;

pub mod ledger;
pub mod runner;

pub use ledger::{const_parse_u32, ledger_storage_init, Ledger, LedgerStorage};
pub use runner::{CodeMem, LoadError, PayloadWatchdog, Runner, Transport, PAYLOAD_WATCHDOG_MS};
