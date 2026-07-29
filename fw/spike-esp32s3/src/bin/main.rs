//! ESP32-S3 dynamic-code feasibility spike for lightplayer's Xtensa backend.
//!
//! Runs the experiment ladder E1..E5 in order, printing machine-checkable
//! serial lines: `En: PASS key=value ...` / `En: FAIL reason=...` /
//! `En: MEASURE key=value`. See README.md and FINDINGS.md.

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
    let _peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 200 * 1024);

    // E1: toolchain + HAL + USB-Serial-JTAG logging alive.
    esp_println::println!(
        "E1: PASS esp_hal={} heap_free={}",
        ESP_HAL_VERSION,
        esp_alloc::HEAP.free()
    );

    let boot = spike_esp32s3::e5::boot_ledger();

    spike_esp32s3::e2::run();
    spike_esp32s3::e3::run();
    spike_esp32s3::e4::run();
    spike_esp32s3::e5::measure();

    if boot == 1 {
        // Give the serial monitor a moment, then prove the recovery tier.
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(1500) {}
        spike_esp32s3::e5::intentional_panic();
    }

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(1000) {}
        esp_println::println!("spike: idle heap_free={}", esp_alloc::HEAP.free());
    }
}
