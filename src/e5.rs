//! E5 — abort-tier recovery sketch + resource measurements.
//!
//! Xtensa has no `unwinding`-crate support, so lightplayer's S3 firmware will
//! use a degraded recovery tier: a custom panic handler that records blame
//! into RTC fast RAM (which survives software resets) and reboots. This module
//! sketches that tier end-to-end once: boot 1 runs the experiments, then
//! panics on purpose; boot 2 proves the ledger survived and reports it.
//!
//! The ledger is `#[ram(rtc_fast, unstable(persistent))]` atomics. RTC RAM
//! also survives *reflashing* (power stays up), so a per-build id from
//! build.rs distinguishes "fresh flash" from "post-panic reboot".

use portable_atomic::{AtomicU32, Ordering};

const BUILD_ID: u32 = const_parse_u32(env!("SPIKE_BUILD_ID"));
const PANIC_CODE_INTENTIONAL: u32 = 0xDEAD_0001;

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_BUILD_ID: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_BOOT_COUNT: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_PANIC_CODE: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_PANIC_LINE: AtomicU32 = AtomicU32::new(0);

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

/// The abort-tier panic handler: record blame, print, reboot.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let line = info.location().map(|l| l.line()).unwrap_or(0);
    // If the code wasn't pre-set by an intentional panic site, mark generic.
    let _ = LEDGER_PANIC_CODE.compare_exchange(0, 0xDEAD_FFFF, Ordering::SeqCst, Ordering::SeqCst);
    LEDGER_PANIC_LINE.store(line, Ordering::SeqCst);
    esp_println::println!("PANIC (rebooting, blame recorded): {info}");
    esp_hal::system::software_reset()
}

/// Boot-time ledger handling. Returns the boot number within this build.
pub fn boot_ledger() -> u32 {
    if LEDGER_BUILD_ID.load(Ordering::SeqCst) != BUILD_ID {
        // Fresh flash: RTC RAM contents belong to a previous build.
        LEDGER_BUILD_ID.store(BUILD_ID, Ordering::SeqCst);
        LEDGER_BOOT_COUNT.store(0, Ordering::SeqCst);
        LEDGER_PANIC_CODE.store(0, Ordering::SeqCst);
        LEDGER_PANIC_LINE.store(0, Ordering::SeqCst);
    }
    let boot = LEDGER_BOOT_COUNT.fetch_add(1, Ordering::SeqCst) + 1;
    if boot >= 2 {
        let code = LEDGER_PANIC_CODE.load(Ordering::SeqCst);
        let line = LEDGER_PANIC_LINE.load(Ordering::SeqCst);
        if code != 0 {
            esp_println::println!(
                "E5: PASS ledger_survived=true boot_count={boot} prev_code={code:#x} prev_line={line}"
            );
        } else {
            esp_println::println!("E5: FAIL boot_count={boot} but no panic recorded");
        }
    } else {
        esp_println::println!("E5: boot 1 of build {BUILD_ID}; will panic intentionally after measurements");
    }
    boot
}

/// Trigger the intentional panic (boot 1 only).
pub fn intentional_panic() -> ! {
    LEDGER_PANIC_CODE.store(PANIC_CODE_INTENTIONAL, Ordering::SeqCst);
    panic!("E5 intentional panic (code {PANIC_CODE_INTENTIONAL:#x})");
}

/// Heap measurements, printed as MEASURE lines.
pub fn measure() {
    let free_at_start = esp_alloc::HEAP.free();
    esp_println::println!("E5: MEASURE heap_free={free_at_start}");

    // A 64KB "JIT arena" — the kind of block the shader engine will want.
    // (Volatile write + black_box: without them LTO elides the unused
    // alloc/dealloc pair entirely and the numbers are fiction.)
    let layout = core::alloc::Layout::from_size_align(64 * 1024, 4).expect("layout");
    // SAFETY: non-zero layout; null-checked below; freed before return.
    let arena = core::hint::black_box(unsafe { alloc::alloc::alloc(layout) });
    if arena.is_null() {
        esp_println::println!("E5: MEASURE arena_64k=FAILED");
    } else {
        // SAFETY: arena is a live allocation of at least 64KB.
        unsafe { arena.write_volatile(0xAA) };
        esp_println::println!(
            "E5: MEASURE arena_64k=ok heap_free_after={}",
            esp_alloc::HEAP.free()
        );
        // SAFETY: allocated just above with the same layout.
        unsafe { alloc::alloc::dealloc(arena, layout) };
    }

    // Largest single allocatable block, by bisection over raw alloc.
    let mut lo = 0usize;
    let mut hi = free_at_start + 4096;
    while hi - lo > 1024 {
        let mid = (lo + hi) / 2;
        let l = core::alloc::Layout::from_size_align(mid, 4).expect("layout");
        // SAFETY: non-zero layout; null-checked; freed immediately.
        let p = core::hint::black_box(unsafe { alloc::alloc::alloc(l) });
        if p.is_null() {
            hi = mid;
        } else {
            // SAFETY: p is a live allocation of at least mid bytes.
            unsafe { p.write_volatile(0xAA) };
            // SAFETY: allocated just above with layout l.
            unsafe { alloc::alloc::dealloc(p, l) };
            lo = mid;
        }
    }
    esp_println::println!("E5: MEASURE largest_block~={lo}");
}
