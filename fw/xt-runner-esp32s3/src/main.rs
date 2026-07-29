//! xt-runner-esp32s3 — resident ESP32-S3 firmware that executes Xtensa code
//! payloads sent over USB-Serial-JTAG, without reflashing.
//!
//! The hardware oracle for the standalone Xtensa core: the host sends a
//! `LoadExec` payload, the runner copies it into an executable buffer, calls it,
//! and replies with the result — or, if the payload faults/hangs, a structured
//! `CrashReport` (delivered on the next boot after an auto-reset). See
//! `xt-runner-proto` for the wire format and README.md for the crash model.
//!
//! All board-agnostic logic (ledger, protocol dispatch, payload flow, watchdog
//! policy) lives in `xt-runner-core`; this crate supplies only the S3 parts:
//! the USB-Serial-JTAG transport, the heap-alias code memory, the RWDT
//! watchdog, the persistent RTC-RAM ledger statics, and the panic handler.
//!
//! The protocol channel is pure binary (COBS-framed postcard); nothing else may
//! write to the USB serial FIFO or it would corrupt framing — hence no
//! esp-println.

#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

mod board;
mod jitbuf;

extern crate alloc;

use esp_hal::rtc_cntl::Rtc;
use esp_hal::system::software_reset;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_hal::{clock::CpuClock, main};

use board::{HeapAliasCodeMem, RwdtWatchdog, SerialTransport};
use xt_runner_core::{const_parse_u32, ledger_storage_init, Ledger, LedgerStorage, Runner};

esp_bootloader_esp_idf::esp_app_desc!();

const BUILD_ID: u32 = const_parse_u32(env!("XT_BUILD_ID"));

/// The crash ledger's persistent cells. Placement is this crate's job (the
/// attribute and the RTC fast region are chip-specific); the logic is core's.
/// Survives resets — including the payload watchdog's ResetCore — and
/// reflashes (power stays up), which is why `BUILD_ID` exists.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_CELLS: LedgerStorage = ledger_storage_init();

/// Cheap `Copy` handle over the cells, shared by `main` and the panic handler.
fn ledger() -> Ledger {
    Ledger::new(&LEDGER_CELLS, BUILD_ID)
}

/// Panic handler. esp-hal's exception handler turns hardware faults into panics,
/// so a crashing payload arrives here. The EXCCAUSE/EPC1/EXCVADDR special
/// registers still hold the last fault, so we recover the precise cause without
/// overriding esp-hal's `__user_exception` (which it defines strongly). If a
/// payload was armed we blame it; then reset so the runner comes back and
/// reports the crash on next boot. (No printing — the channel is binary.)
///
/// EPC1 is the real faulting PC (exception frames aren't window-mangled — only
/// a0 return addresses are — so no unmangling is needed).
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let (exccause, epc1, excvaddr): (u32, u32, u32);
    // SAFETY: rsr reads of special registers; no memory or state effects.
    unsafe {
        core::arch::asm!(
            "rsr.exccause {0}",
            "rsr.epc1 {1}",
            "rsr.excvaddr {2}",
            out(reg) exccause,
            out(reg) epc1,
            out(reg) excvaddr,
        );
    }
    // Only transitions RUNNING→CRASHED, so an idle panic is not blamed on a
    // stale payload seq.
    ledger().record_crash(exccause, epc1, excvaddr);
    software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 200 * 1024);

    let rtc = Rtc::new(peripherals.LPWR);
    let serial = UsbSerialJtag::new(peripherals.USB_DEVICE);

    Runner::new(
        SerialTransport(serial),
        HeapAliasCodeMem::new(),
        RwdtWatchdog(rtc),
        ledger(),
        xt_runner_proto::Chip::Esp32S3,
        || esp_alloc::HEAP.free() as u32,
    )
    .run()
}
