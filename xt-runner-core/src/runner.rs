//! The board-agnostic runner: COBS frame accumulation, `Request` → `Response`
//! dispatch, and the payload flow (load → arm → sync → call → disarm) with the
//! watchdog policy around it.
//!
//! The protocol channel is pure binary (COBS-framed postcard); nothing else may
//! write to the transport or it would corrupt framing — hence no esp-println in
//! any firmware built on this.

use alloc::vec::Vec;

use xt_runner_proto::{Chip, DeviceInfo, ErrorCode, Request, Response, MAX_PAYLOAD, PROTO_VERSION};

use crate::ledger::Ledger;

/// Watchdog window for a single payload. Generous — payloads are tiny; this only
/// catches genuine hangs (infinite loops in emitted code).
pub const PAYLOAD_WATCHDOG_MS: u64 = 3000;

/// The serial channel to the host (USB-Serial-JTAG on S3, a UART bridge on
/// classic ESP32).
pub trait Transport {
    /// Next received byte, or `None` if nothing is pending. Transports handle
    /// their own line errors internally (drop the byte); the framing layer
    /// resynchronises on the next COBS delimiter regardless.
    fn read_byte(&mut self) -> Option<u8>;
    /// Queue `bytes` for transmission (blocking is fine).
    fn write(&mut self, bytes: &[u8]);
    /// Block until queued bytes are on the wire.
    fn flush(&mut self);
}

/// Why a payload could not be placed in executable memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadError {
    /// The payload does not fit the board's code region.
    TooLarge { len: usize, capacity: usize },
}

/// Board-specific executable memory for dynamically written code.
///
/// The write path and the execute address are deliberately decoupled, because
/// the relationship between them is a per-chip discovery, not a constant:
///
/// - **ESP32-S3**: code is written to a heap buffer through its D-bus address
///   and executed at the uniform `+0x6F_0000` I-bus alias.
/// - **Classic ESP32** (see FINDINGS.md, C2): the heap is *not* executable.
///   Code goes to SRAM1, whose dual mapping is word-**mirrored**
///   (`iram = 0x400B_FFFC − (dram − 0x3FFE_0000)` — the D-bus write walk runs
///   *backwards* through memory), or to SRAM0 which allows 32-bit-aligned word
///   writes only, or to an 8KB RTC region. The returned I-bus address is not
///   related to any write address by a simple offset.
///
/// Hence `load` takes the whole payload and is free to write it word-by-word
/// in any address order; only the returned entry base is meaningful to the
/// caller. A fixed-size region reports `LoadError::TooLarge`, and
/// [`capacity`](CodeMem::capacity) lets the host size payloads.
pub trait CodeMem {
    /// Copy `code` into executable memory; return the I-bus (execute) address
    /// of its first byte. The code must stay loaded until [`release`]
    /// (CodeMem::release) or the next `load`.
    fn load(&mut self, code: &[u8]) -> Result<usize, LoadError>;

    /// Instruction-fetch/memory barriers after writing code, before executing
    /// it (`memw` + `isync` on both measured chips — strictly belt-and-braces,
    /// internal SRAM is uncached, but the cost is nil).
    fn sync(&mut self);

    /// Release per-payload resources after the payload has returned. Default
    /// no-op for fixed-region implementations.
    fn release(&mut self) {}

    /// Largest payload this board's code region can hold, in bytes.
    fn capacity(&self) -> usize;
}

/// The hang-recovery watchdog armed around each payload call.
///
/// Implementations MUST configure the reset action as a **core-only reset**
/// (esp-hal `RwdtStageAction::ResetCore`), never the `enable()` default
/// `ResetSystem`: a system reset ALSO resets the RTC peripherals — wiping the
/// RTC-RAM crash ledger, so a hang would look like a fresh flash and no
/// timeout would be reported. A core reset leaves RTC RAM intact. Use
/// [`PAYLOAD_WATCHDOG_MS`] as the timeout.
pub trait PayloadWatchdog {
    /// Arm the watchdog for one payload window.
    fn arm(&mut self);
    /// Disarm after a clean payload return.
    fn disarm(&mut self);
}

/// The resident payload runner. Construct once at boot, then [`run`](Runner::run).
pub struct Runner<T: Transport, C: CodeMem, W: PayloadWatchdog> {
    transport: T,
    codemem: C,
    watchdog: W,
    ledger: Ledger,
    /// The SOC this firmware was built for, reported in `DeviceInfo::chip`.
    chip: Chip,
    /// Board hook for `DeviceInfo::heap_free` (e.g. `esp_alloc::HEAP.free()`).
    heap_free: fn() -> u32,
    boot_count: u32,
    pending_crash: Option<xt_runner_proto::CrashReport>,
    rx: Vec<u8>,
}

impl<T: Transport, C: CodeMem, W: PayloadWatchdog> Runner<T, C, W> {
    /// Reads the ledger (`Ledger::boot`) — call once per boot, after the
    /// firmware has initialised the heap and peripherals.
    pub fn new(
        transport: T,
        codemem: C,
        watchdog: W,
        ledger: Ledger,
        chip: Chip,
        heap_free: fn() -> u32,
    ) -> Self {
        let (boot_count, pending_crash) = ledger.boot();
        Runner {
            transport,
            codemem,
            watchdog,
            ledger,
            chip,
            heap_free,
            boot_count,
            pending_crash,
            rx: Vec::with_capacity(1024),
        }
    }

    /// Serve the protocol forever. Crashing payloads do not return here — they
    /// reset the chip (fault → panic handler, hang → watchdog) and the next
    /// boot's `Runner::new` picks the report up from the ledger.
    pub fn run(mut self) -> ! {
        // Announce any crash from the payload that reset us last boot, unsolicited.
        if let Some(report) = self.pending_crash.take() {
            self.send(&Response::Crash(report));
        }

        loop {
            match self.transport.read_byte() {
                Some(0x00) => {
                    // COBS frame delimiter — decode and handle.
                    if !self.rx.is_empty() {
                        self.handle_frame();
                        self.rx.clear();
                    }
                }
                Some(b) => {
                    if self.rx.len() < MAX_PAYLOAD + 512 {
                        self.rx.push(b);
                    } else {
                        // Overrun — drop the partial frame, resync on next delimiter.
                        self.rx.clear();
                    }
                }
                None => { /* spin */ }
            }
        }
    }

    fn handle_frame(&mut self) {
        // `rx` holds the COBS bytes without the trailing delimiter; re-append it
        // for postcard's cobs decoder, which expects the sentinel.
        let mut buf = Vec::with_capacity(self.rx.len() + 1);
        buf.extend_from_slice(&self.rx);
        buf.push(0);

        let req: Request = match xt_runner_proto::decode(&mut buf) {
            Ok(r) => r,
            Err(_) => return, // undecodable — ignore, host will time out and retry
        };

        match req {
            Request::Ping => self.send(&Response::Pong),
            Request::Info => {
                let info = Response::Info(DeviceInfo {
                    proto_version: PROTO_VERSION,
                    chip: self.chip,
                    heap_free: (self.heap_free)(),
                    // The protocol cap, further bounded by the board's code
                    // region (identical on S3, where the region is heap-backed).
                    max_payload: MAX_PAYLOAD.min(self.codemem.capacity()) as u32,
                    boot_count: self.boot_count,
                });
                self.send(&info);
            }
            Request::LoadExec {
                seq,
                entry_offset,
                arg,
                code,
            } => {
                if code.len() > MAX_PAYLOAD {
                    self.send(&Response::Error { seq, code: ErrorCode::PayloadTooLarge });
                    return;
                }
                if (entry_offset as usize) >= code.len() {
                    self.send(&Response::Error { seq, code: ErrorCode::BadEntryOffset });
                    return;
                }
                match self.run_payload(seq, &code, entry_offset, arg) {
                    Ok(result) => self.send(&Response::Ok { seq, result }),
                    Err(LoadError::TooLarge { .. }) => {
                        self.send(&Response::Error { seq, code: ErrorCode::PayloadTooLarge })
                    }
                }
            }
        }
    }

    /// Copy `code` into executable memory and call it. If it faults or hangs,
    /// the exception handler / watchdog resets the chip and the NEXT boot
    /// reports the crash — so this function only returns on success (or a
    /// payload the board's code region cannot hold).
    fn run_payload(&mut self, seq: u32, code: &[u8], entry_offset: u32, arg: u32) -> Result<u32, LoadError> {
        let entry = self.codemem.load(code)? + entry_offset as usize;

        self.ledger.arm(seq);
        self.watchdog.arm();
        self.codemem.sync();

        // SAFETY: `entry` is the I-bus (execute) address of freshly written,
        // synced code holding a complete windowed function
        // `extern "C" fn(u32) -> u32`. A malformed payload faults into the
        // exception path (→ reset) or hangs into the watchdog (→ reset); either
        // way control does not return here corrupted.
        let f: extern "C" fn(u32) -> u32 = unsafe { core::mem::transmute(entry) };
        let result = f(arg);

        self.watchdog.disarm();
        self.ledger.disarm();
        self.codemem.release();
        Ok(result)
    }

    fn send(&mut self, resp: &Response) {
        if let Ok(bytes) = xt_runner_proto::encode(resp) {
            // Leading COBS delimiter: on reset the ROM bootloader prints text to
            // this same channel, which would otherwise be prepended to our frame.
            // The leading 0x00 isolates that noise into its own (empty/garbage)
            // frame the host skips, so the real frame after it decodes cleanly.
            self.transport.write(&[0x00]);
            self.transport.write(&bytes);
            self.transport.flush();
        }
    }
}
