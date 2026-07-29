//! Classic-ESP32 (LX6) dynamic-code experiment ladder for lightplayer's
//! Xtensa backend — mirrors the S3 spike's E1–E5 as C1–C5.
//!
//! Runs C1..C5 in order, printing machine-checkable serial lines over UART0
//! (through the board's USB-UART bridge, 115200 baud):
//! `Cn: PASS key=value ...` / `Cn: FAIL reason=...` / `Cn: MEASURE key=value`.
//! See the repo FINDINGS.md for the verdicts.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::{
    clock::CpuClock,
    main,
    time::{Duration, Instant},
};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

/// Recorded from Cargo.lock; verified in FINDINGS.md.
const ESP_HAL_VERSION: &str = "1.1.1";

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Classic-ESP32 gotcha: esp-println's `uart` feature writes the UART0 TX
    // FIFO directly and never programs the baud divisor. The ROM's divisor is
    // stale once esp_hal::init() reclocks the chip (output turns to garbage),
    // so program UART0 to 115200 for the current clock tree ourselves. The
    // binding stays alive for the whole of `main` (which never returns), so
    // the configuration persists; esp-println keeps using the raw FIFO.
    let _uart0_tx = esp_hal::uart::UartTx::new(
        peripherals.UART0,
        esp_hal::uart::Config::default(), // 115200 8N1
    )
    .expect("uart0 config")
    .with_tx(peripherals.GPIO1);

    // dram_seg on classic is only 192KB (SRAM2, minus stack + statics), vs
    // the S3's 345KB — a 96KB heap leaves comfortable stack headroom.
    esp_alloc::heap_allocator!(size: 96 * 1024);

    // C1: toolchain + HAL + UART logging alive.
    esp_println::println!(
        "C1: PASS esp_hal={} chip=esp32 heap_free={}",
        ESP_HAL_VERSION,
        esp_alloc::HEAP.free()
    );

    let boot = spike_esp32::c5::boot_ledger();

    // Sacrificial fault probes (feature-gated): run first so the exception
    // context prints immediately; each faults -> panics -> reboots, so a
    // probe build reboot-loops by design.
    #[cfg(feature = "probe-iram-byte")]
    spike_esp32::c2::fault_probe_iram_byte();
    #[cfg(feature = "probe-identity-exec")]
    spike_esp32::c2::fault_probe_identity_exec();

    let outcome = spike_esp32::c2::run();
    if let Some(kind) = outcome.primary {
        spike_esp32::c3::run(kind);
        spike_esp32::c4::run(kind);
    } else {
        esp_println::println!("C3: FAIL reason=no_executable_region");
        esp_println::println!("C4: FAIL reason=no_executable_region");
    }
    spike_esp32::c5::measure();

    if boot == 1 {
        // Give the serial monitor a moment, then prove the recovery tier.
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(1500) {}
        spike_esp32::c5::intentional_panic();
    }

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(1000) {}
        esp_println::println!("spike-esp32: idle heap_free={}", esp_alloc::HEAP.free());
    }
}
