//! Crash ledger in RTC fast RAM (survives resets, incl. watchdog resets) plus
//! the custom exception/panic handlers that populate it.
//!
//! Flow: before jumping into a payload, `arm(seq)` records RUNNING + seq. On a
//! clean return, `disarm()` sets IDLE. If the payload faults, `__user_exception`
//! records EXC + cause/pc/vaddr and resets. If it panics in Rust, the panic
//! handler records PANIC and resets. If it hangs, the watchdog resets with the
//! ledger still RUNNING — which the next boot reads as a TIMEOUT.
//!
//! RTC RAM also survives reflashing (power stays up), so a build-id from
//! build.rs distinguishes a fresh flash from a post-crash reboot.

use portable_atomic::{AtomicU32, Ordering};
use xt_runner_proto::{CrashKind, CrashReport};

const BUILD_ID: u32 = const_parse_u32(env!("XT_BUILD_ID"));

// State values.
const IDLE: u32 = 0;
const RUNNING: u32 = 1;
const CRASHED: u32 = 2;

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_BUILD_ID: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_BOOT_COUNT: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_STATE: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_SEQ: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_CAUSE: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_PC: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static L_VADDR: AtomicU32 = AtomicU32::new(0);

const fn const_parse_u32(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut v: u32 = 0;
    let mut i = 0;
    while i < bytes.len() {
        v = v.wrapping_mul(10).wrapping_add((bytes[i] - b'0') as u32);
        i += 1;
    }
    v
}

/// Call once at boot. Resets the ledger on a fresh flash; returns the boot count
/// and any pending crash from the previous boot (to report to the host).
pub fn boot() -> (u32, Option<CrashReport>) {
    if L_BUILD_ID.load(Ordering::SeqCst) != BUILD_ID {
        L_BUILD_ID.store(BUILD_ID, Ordering::SeqCst);
        L_BOOT_COUNT.store(0, Ordering::SeqCst);
        L_STATE.store(IDLE, Ordering::SeqCst);
    }
    let boot_count = L_BOOT_COUNT.fetch_add(1, Ordering::SeqCst) + 1;

    let report = match L_STATE.swap(IDLE, Ordering::SeqCst) {
        RUNNING => Some(CrashReport {
            // RUNNING survived a reset → the watchdog fired: the payload hung.
            seq: L_SEQ.load(Ordering::SeqCst),
            kind: CrashKind::Timeout,
            cause: 0,
            pc: 0,
            vaddr: 0,
        }),
        CRASHED => {
            let cause = L_CAUSE.load(Ordering::SeqCst);
            Some(CrashReport {
                seq: L_SEQ.load(Ordering::SeqCst),
                // A real EXCCAUSE (a hardware fault routed through esp-hal's
                // handler) vs a plain Rust panic in the runner path.
                kind: if is_exception_cause(cause) {
                    CrashKind::Exception
                } else {
                    CrashKind::Panic
                },
                cause,
                pc: L_PC.load(Ordering::SeqCst),
                vaddr: L_VADDR.load(Ordering::SeqCst),
            })
        }
        _ => None,
    };
    (boot_count, report)
}

/// Record that payload `seq` is about to run.
pub fn arm(seq: u32) {
    L_SEQ.store(seq, Ordering::SeqCst);
    L_STATE.store(RUNNING, Ordering::SeqCst);
}

/// Record that the running payload returned cleanly.
pub fn disarm() {
    L_STATE.store(IDLE, Ordering::SeqCst);
}

/// Record a crash (from the panic handler, with the fault special-registers).
/// Only meaningful while a payload is armed; a runner-internal panic still
/// records so the device resets cleanly rather than hanging.
pub fn record_crash(cause: u32, pc: u32, vaddr: u32) {
    L_CAUSE.store(cause, Ordering::SeqCst);
    L_PC.store(pc, Ordering::SeqCst);
    L_VADDR.store(vaddr, Ordering::SeqCst);
    // Only transition RUNNING→CRASHED; a panic outside payload execution leaves
    // state IDLE so boot() doesn't misreport it against a stale seq.
    let _ = L_STATE.compare_exchange(RUNNING, CRASHED, Ordering::SeqCst, Ordering::SeqCst);
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
