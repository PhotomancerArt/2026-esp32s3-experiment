//! xt-runner-esp32 — resident classic-ESP32 (LX6) firmware that executes
//! Xtensa code payloads sent over UART0 (through the board's USB-UART
//! bridge), without reflashing.
//!
//! The classic sibling of `fw/xt-runner-esp32s3`: all board-agnostic logic
//! (ledger, protocol dispatch, payload flow, watchdog policy) lives in
//! `xt-runner-core`; this crate supplies only the classic parts — the UART0
//! transport, the fixed mirrored-SRAM1 code memory (the classic heap is NOT
//! executable; see `codemem`), the RWDT watchdog, the persistent RTC-RAM
//! ledger statics, and the panic handler.
//!
//! The protocol channel is pure binary (COBS-framed postcard) on UART0 at
//! 115200; nothing else may write to that FIFO or it would corrupt framing —
//! hence no esp-println anywhere in this crate (which also sidesteps the C1
//! stale-baud-divisor gotcha: our own `Uart::new` after `esp_hal::init()`
//! programs the divisor for the reclocked chip).

#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

mod board;
mod codemem;

extern crate alloc;

use esp_hal::rtc_cntl::Rtc;
use esp_hal::system::software_reset;
use esp_hal::uart::Uart;
use esp_hal::{clock::CpuClock, main};

use board::{RwdtWatchdog, UartTransport};
use codemem::Sram1CodeMem;
use xt_runner_core::{const_parse_u32, ledger_storage_init, Ledger, LedgerStorage, Runner};

esp_bootloader_esp_idf::esp_app_desc!();

const BUILD_ID: u32 = const_parse_u32(env!("XT_BUILD_ID"));

/// The crash ledger's persistent cells. Placement is this crate's job (the
/// attribute and the RTC fast region are chip-specific — on classic: 8KB at
/// DRAM `0x3FF8_0000`, PRO_CPU only; C5 proved the persistent attribute works
/// unchanged); the logic is core's. Survives resets — including the payload
/// watchdog's ResetCore — and reflashes (power stays up), hence `BUILD_ID`.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_CELLS: LedgerStorage = ledger_storage_init();

/// Cheap `Copy` handle over the cells, shared by `main` and the panic handler.
fn ledger() -> Ledger {
    Ledger::new(&LEDGER_CELLS, BUILD_ID)
}

/// Panic handler. esp-hal's exception handler turns hardware faults into
/// panics, so a crashing payload arrives here. The EXCCAUSE/EPC1/EXCVADDR
/// special registers still hold the last fault, so we recover the precise
/// cause. If a payload was armed we blame it; then reset so the runner comes
/// back and reports the crash on next boot. (No printing — the channel is
/// binary.)
///
/// EPC1 is the real faulting PC (exception frames aren't window-mangled —
/// only a0 return addresses are — so no unmangling is needed).
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
    // Classic-ESP32 gotcha (C5, found on hardware): software_reset() kills an
    // undrained UART0 TX FIFO — anything queued but not yet on the wire is
    // truncated (the S3's USB-CDC was immune). The runner flushes after every
    // send, so this is defensive — it covers a panic that lands mid-`send`
    // (bytes written, flush not reached) so the partial frame at least
    // drains and the host resyncs on a clean boundary. 300ms covers a full
    // TX FIFO several times over at 115200.
    let start = esp_hal::time::Instant::now();
    while start.elapsed() < esp_hal::time::Duration::from_millis(300) {}
    software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    // dram_seg on classic is only 192KB (SRAM2) vs the S3's 345KB; 96KB
    // leaves comfortable stack headroom, and payloads do NOT come from this
    // heap (the classic heap is not executable — code goes to the fixed
    // SRAM1 region, see `codemem`).
    esp_alloc::heap_allocator!(size: 96 * 1024);

    let rtc = Rtc::new(peripherals.LPWR);
    // UART0 through the USB-UART bridge, 115200 8N1 (Config::default()),
    // on the classic devkit's UART0 pins (TX=GPIO1, RX=GPIO3). Constructed
    // after esp_hal::init() so the baud divisor matches the reclocked chip.
    let uart = Uart::new(peripherals.UART0, esp_hal::uart::Config::default())
        .expect("uart0 config")
        .with_tx(peripherals.GPIO1)
        .with_rx(peripherals.GPIO3);

    Runner::new(
        UartTransport::new(uart),
        Sram1CodeMem::new(),
        RwdtWatchdog(rtc),
        ledger(),
        xt_runner_proto::Chip::Esp32,
        || esp_alloc::HEAP.free() as u32,
    )
    .run()
}
