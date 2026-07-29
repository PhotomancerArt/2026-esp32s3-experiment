//! Wire protocol for `xt-runner`: send Xtensa code payloads to a resident
//! device firmware (ESP32-S3 over USB-Serial-JTAG, classic ESP32 over its
//! UART bridge), execute them without reflashing, and get results or
//! structured crash reports back. `DeviceInfo::chip` tells the host which
//! board it is actually talking to, so a harness can verify a port matches
//! the board it was configured for.
//!
//! Frames are COBS-encoded postcard (zero byte = frame delimiter), so the host
//! can resynchronise after the device resets mid-frame following a crash.
//!
//! `no_std + alloc` so the same types compile into the firmware and the host
//! client. This is original code (no derivation); see the repo license ADR.

#![no_std]

extern crate alloc;

use alloc::vec::Vec;
use serde::{Deserialize, Serialize};

/// Bumped when the wire format changes incompatibly.
/// v2: `DeviceInfo` gained `chip`.
pub const PROTO_VERSION: u32 = 2;

/// Host → device.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Request {
    /// Liveness check.
    Ping,
    /// Ask for firmware/board info.
    Info,
    /// Copy `code` into an executable buffer and call
    /// `(buffer + entry_offset)(arg)` as `extern "C" fn(u32) -> u32`.
    LoadExec {
        /// Caller-chosen id, echoed in the reply and in any crash report so the
        /// host learns which payload killed the device across a reset.
        seq: u32,
        /// Byte offset of the entry point within `code` (e.g. past a literal pool).
        entry_offset: u32,
        /// Single u32 argument, staged per the windowed ABI (callee `a2`).
        arg: u32,
        code: Vec<u8>,
    },
}

/// Device → host.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Response {
    Pong,
    Info(DeviceInfo),
    /// A payload ran to completion.
    Ok { seq: u32, result: u32 },
    /// A payload crashed or hung; delivered either as the direct reply or,
    /// after a reset, unsolicited on the next boot.
    Crash(CrashReport),
    /// The request could not be handled (e.g. payload too large).
    Error { seq: u32, code: ErrorCode },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ErrorCode {
    /// `code.len()` exceeded `MAX_PAYLOAD`.
    PayloadTooLarge,
    /// `entry_offset` was outside the payload.
    BadEntryOffset,
}

/// Which SOC the firmware was built for. Reported by the device (each
/// firmware crate hardcodes its own), never told to it — so the host can
/// detect a port wired to the wrong board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Chip {
    /// ESP32-S3 (Xtensa LX7).
    Esp32S3,
    /// Classic ESP32 (Xtensa LX6).
    Esp32,
}

impl core::fmt::Display for Chip {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Chip::Esp32S3 => "esp32s3",
            Chip::Esp32 => "esp32",
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub proto_version: u32,
    /// The SOC this firmware runs on.
    pub chip: Chip,
    pub heap_free: u32,
    pub max_payload: u32,
    pub boot_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CrashReport {
    pub seq: u32,
    pub kind: CrashKind,
    /// EXCCAUSE for exceptions, else 0.
    pub cause: u32,
    /// Faulting PC, window-mangle bits already cleared. 0 if unknown.
    pub pc: u32,
    /// EXCVADDR for load/store faults, else 0.
    pub vaddr: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CrashKind {
    /// A hardware exception (bad fetch, load/store error, illegal instr, ...).
    Exception,
    /// A Rust panic inside the runner while handling the payload.
    Panic,
    /// The watchdog fired — the payload hung.
    Timeout,
}

/// Max payload bytes the device accepts (keeps the RX buffer bounded).
pub const MAX_PAYLOAD: usize = 32 * 1024;

/// Encode a message as a COBS-framed postcard buffer (trailing 0 delimiter).
pub fn encode<T: Serialize>(msg: &T) -> Result<Vec<u8>, postcard::Error> {
    postcard::to_allocvec_cobs(msg)
}

/// Decode one COBS-framed postcard message from `frame` (delimiter already
/// stripped by the caller's accumulator, or included — postcard tolerates it).
pub fn decode<'a, T: Deserialize<'a>>(frame: &'a mut [u8]) -> Result<T, postcard::Error> {
    postcard::from_bytes_cobs(frame)
}
