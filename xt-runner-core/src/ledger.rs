//! Crash-ledger logic for RTC fast RAM (survives resets, incl. watchdog resets).
//!
//! Flow: before jumping into a payload, `arm(seq)` records RUNNING + seq. On a
//! clean return, `disarm()` sets IDLE. If the payload faults, the firmware's
//! panic handler records EXC cause/pc/vaddr and resets. If it panics in Rust,
//! the same path records it as a PANIC. If it hangs, the watchdog resets with
//! the ledger still RUNNING — which the next boot reads as a TIMEOUT.
//!
//! RTC RAM also survives reflashing (power stays up), so a build-id from
//! build.rs distinguishes a fresh flash from a post-crash reboot.
//!
//! ## Storage boundary
//!
//! The *placement* of the ledger is chip-specific: the RTC fast region differs
//! between SOCs (S3: 8KB dual-mapped at its own addresses; classic ESP32: 8KB
//! at DRAM `0x3FF8_0000`, PRO_CPU only) and the persistent-static attribute is
//! esp-hal's. So this module owns only the logic, over a [`LedgerStorage`]
//! array the firmware declares:
//!
//! ```ignore
//! #[esp_hal::ram(unstable(rtc_fast, persistent))]
//! static LEDGER_CELLS: LedgerStorage = ledger_storage_init();
//! ```
//!
//! `LedgerStorage` is a plain `[AtomicU32; N]` (not a struct) because esp-hal's
//! `persistent` attribute requires the static's type to implement its sealed
//! `Persistable` trait — implemented for atomics and arrays of them, and not
//! implementable for a foreign struct from here (orphan rule).

use portable_atomic::{AtomicU32, Ordering};
use xt_runner_proto::{CrashKind, CrashReport};

// Cell indices within `LedgerStorage`.
const BUILD_ID: usize = 0;
const BOOT_COUNT: usize = 1;
const STATE: usize = 2;
const SEQ: usize = 3;
const CAUSE: usize = 4;
const PC: usize = 5;
const VADDR: usize = 6;
/// Number of ledger cells.
pub const NUM_CELLS: usize = 7;

// State values.
const IDLE: u32 = 0;
const RUNNING: u32 = 1;
const CRASHED: u32 = 2;

/// The persistent backing store for a [`Ledger`]. The firmware declares one of
/// these as a static in persistent RTC RAM (see the module docs).
pub type LedgerStorage = [AtomicU32; NUM_CELLS];

/// Initializer for a [`LedgerStorage`] static (a `const fn` rather than a
/// named const so the interior-mutable cells are never copied out of a const).
pub const fn ledger_storage_init() -> LedgerStorage {
    [const { AtomicU32::new(0) }; NUM_CELLS]
}

/// Parse a decimal `u32` in const context (for `env!("XT_BUILD_ID")`).
pub const fn const_parse_u32(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut v: u32 = 0;
    let mut i = 0;
    while i < bytes.len() {
        v = v.wrapping_mul(10).wrapping_add((bytes[i] - b'0') as u32);
        i += 1;
    }
    v
}

/// A cheap `Copy` handle over the persistent cells, usable from both `main`
/// and the panic handler.
#[derive(Clone, Copy)]
pub struct Ledger {
    cells: &'static LedgerStorage,
    build_id: u32,
}

impl Ledger {
    pub const fn new(cells: &'static LedgerStorage, build_id: u32) -> Ledger {
        Ledger { cells, build_id }
    }

    /// Call once at boot. Resets the ledger on a fresh flash; returns the boot
    /// count and any pending crash from the previous boot (to report to the
    /// host).
    pub fn boot(&self) -> (u32, Option<CrashReport>) {
        if self.cells[BUILD_ID].load(Ordering::SeqCst) != self.build_id {
            self.cells[BUILD_ID].store(self.build_id, Ordering::SeqCst);
            self.cells[BOOT_COUNT].store(0, Ordering::SeqCst);
            self.cells[STATE].store(IDLE, Ordering::SeqCst);
        }
        let boot_count = self.cells[BOOT_COUNT].fetch_add(1, Ordering::SeqCst) + 1;

        let report = match self.cells[STATE].swap(IDLE, Ordering::SeqCst) {
            RUNNING => Some(CrashReport {
                // RUNNING survived a reset → the watchdog fired: the payload hung.
                seq: self.cells[SEQ].load(Ordering::SeqCst),
                kind: CrashKind::Timeout,
                cause: 0,
                pc: 0,
                vaddr: 0,
            }),
            CRASHED => {
                let cause = self.cells[CAUSE].load(Ordering::SeqCst);
                Some(CrashReport {
                    seq: self.cells[SEQ].load(Ordering::SeqCst),
                    // A real EXCCAUSE (a hardware fault routed through esp-hal's
                    // handler) vs a plain Rust panic in the runner path.
                    kind: if is_exception_cause(cause) {
                        CrashKind::Exception
                    } else {
                        CrashKind::Panic
                    },
                    cause,
                    pc: self.cells[PC].load(Ordering::SeqCst),
                    vaddr: self.cells[VADDR].load(Ordering::SeqCst),
                })
            }
            _ => None,
        };
        (boot_count, report)
    }

    /// Record that payload `seq` is about to run.
    pub fn arm(&self, seq: u32) {
        self.cells[SEQ].store(seq, Ordering::SeqCst);
        self.cells[STATE].store(RUNNING, Ordering::SeqCst);
    }

    /// Record that the running payload returned cleanly.
    pub fn disarm(&self) {
        self.cells[STATE].store(IDLE, Ordering::SeqCst);
    }

    /// Record a crash (from the panic handler, with the fault special-registers).
    /// Only meaningful while a payload is armed; a runner-internal panic still
    /// records so the device resets cleanly rather than hanging.
    pub fn record_crash(&self, cause: u32, pc: u32, vaddr: u32) {
        self.cells[CAUSE].store(cause, Ordering::SeqCst);
        self.cells[PC].store(pc, Ordering::SeqCst);
        self.cells[VADDR].store(vaddr, Ordering::SeqCst);
        // Only transition RUNNING→CRASHED; a panic outside payload execution
        // leaves state IDLE so boot() doesn't misreport it against a stale seq.
        let _ = self.cells[STATE].compare_exchange(RUNNING, CRASHED, Ordering::SeqCst, Ordering::SeqCst);
    }
}

/// Heuristic: does `cause` look like a genuine EXCCAUSE the CPU would set?
/// The Xtensa general-exception causes we care about are 0..=63; a fresh boot
/// with no exception leaves a small value too, but by then state != CRASHED so
/// this is only consulted when a real fault routed through the panic path.
fn is_exception_cause(cause: u32) -> bool {
    // Causes actually produced by bad payloads: Illegal(0), InstrError(2),
    // LoadStoreError(3), and the addr/data-error family up to ~15, plus
    // Unaligned(9), Privileged(8). Treat the documented 0..=63 range as real.
    cause <= 63
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;

    fn fresh(build_id: u32) -> Ledger {
        // Leak per-test storage so each test gets independent 'static cells.
        Ledger::new(Box::leak(Box::new(ledger_storage_init())), build_id)
    }

    #[test]
    fn fresh_flash_resets_then_counts_boots() {
        let l = fresh(42);
        assert_eq!(l.boot(), (1, None));
        assert_eq!(l.boot(), (2, None));
    }

    #[test]
    fn build_id_change_resets_boot_count() {
        let cells: &'static LedgerStorage = Box::leak(Box::new(ledger_storage_init()));
        let old = Ledger::new(cells, 1);
        assert_eq!(old.boot().0, 1);
        assert_eq!(old.boot().0, 2);
        let new = Ledger::new(cells, 2);
        assert_eq!(new.boot(), (1, None));
    }

    #[test]
    fn hang_reads_as_timeout() {
        let l = fresh(7);
        l.boot();
        l.arm(99);
        // (watchdog reset happens here — RUNNING survives)
        let (_, report) = l.boot();
        let report = report.expect("timeout report");
        assert_eq!(report.kind, CrashKind::Timeout);
        assert_eq!(report.seq, 99);
    }

    #[test]
    fn fault_reads_as_exception_and_clean_return_reports_nothing() {
        let l = fresh(7);
        l.boot();
        l.arm(5);
        l.record_crash(3, 0x4009_C100, 0x4009_C001); // LoadStoreError
        let (_, report) = l.boot();
        let report = report.expect("crash report");
        assert_eq!(report.kind, CrashKind::Exception);
        assert_eq!((report.seq, report.cause, report.pc, report.vaddr), (5, 3, 0x4009_C100, 0x4009_C001));

        l.arm(6);
        l.disarm();
        assert!(l.boot().1.is_none());
    }

    #[test]
    fn non_exccause_reads_as_panic_and_idle_panic_is_not_blamed() {
        let l = fresh(7);
        l.boot();
        l.arm(8);
        l.record_crash(0xdead_0001, 0, 0); // Rust panic sentinel, not an EXCCAUSE
        assert_eq!(l.boot().1.expect("report").kind, CrashKind::Panic);

        // A panic while IDLE must not flip the state to CRASHED.
        l.record_crash(2, 0x4000_0000, 0);
        assert!(l.boot().1.is_none());
    }
}
