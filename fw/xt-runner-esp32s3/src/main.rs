//! xt-runner — resident ESP32-S3 firmware that executes Xtensa code payloads
//! sent over USB-Serial-JTAG, without reflashing.
//!
//! The hardware oracle for the standalone Xtensa core: the host sends a
//! `LoadExec` payload, the runner copies it into an executable buffer, calls it,
//! and replies with the result — or, if the payload faults/hangs, a structured
//! `CrashReport` (delivered on the next boot after an auto-reset). See
//! `xt-runner-proto` for the wire format and README.md for the crash model.
//!
//! The protocol channel is pure binary (COBS-framed postcard); nothing else may
//! write to the USB serial FIFO or it would corrupt framing — hence no
//! esp-println.

#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

mod jitbuf;
mod ledger;

extern crate alloc;

use alloc::vec::Vec;

use esp_hal::rtc_cntl::{Rtc, RwdtStage, RwdtStageAction};
use esp_hal::system::software_reset;
use esp_hal::time::Duration;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_hal::{clock::CpuClock, main};

use jitbuf::{sync_code, JitBuf};
use xt_runner_proto::{DeviceInfo, Request, Response, MAX_PAYLOAD, PROTO_VERSION};

esp_bootloader_esp_idf::esp_app_desc!();

/// Watchdog window for a single payload. Generous — payloads are tiny; this only
/// catches genuine hangs (infinite loops in emitted code).
const PAYLOAD_WATCHDOG_MS: u64 = 3000;

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
    ledger::record_crash(exccause, epc1, excvaddr);
    software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 200 * 1024);

    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut serial = UsbSerialJtag::new(peripherals.USB_DEVICE);

    let (boot_count, pending_crash) = ledger::boot();

    // Announce any crash from the payload that reset us last boot, unsolicited.
    if let Some(report) = pending_crash {
        send(&mut serial, &Response::Crash(report));
    }

    let mut rx: Vec<u8> = Vec::with_capacity(1024);
    loop {
        match serial.read_byte() {
            Ok(0x00) => {
                // COBS frame delimiter — decode and handle.
                if !rx.is_empty() {
                    handle_frame(&mut serial, &mut rx, &mut rtc, boot_count);
                    rx.clear();
                }
            }
            Ok(b) => {
                if rx.len() < MAX_PAYLOAD + 512 {
                    rx.push(b);
                } else {
                    // Overrun — drop the partial frame, resync on next delimiter.
                    rx.clear();
                }
            }
            Err(nb::Error::WouldBlock) => { /* spin */ }
            Err(_) => rx.clear(),
        }
    }
}

fn handle_frame(
    serial: &mut UsbSerialJtag<'_, esp_hal::Blocking>,
    frame: &mut [u8],
    rtc: &mut Rtc<'_>,
    boot_count: u32,
) {
    // `frame` holds the COBS bytes without the trailing delimiter; re-append it
    // for postcard's cobs decoder, which expects the sentinel.
    let mut buf = Vec::with_capacity(frame.len() + 1);
    buf.extend_from_slice(frame);
    buf.push(0);

    let req: Request = match xt_runner_proto::decode(&mut buf) {
        Ok(r) => r,
        Err(_) => return, // undecodable — ignore, host will time out and retry
    };

    match req {
        Request::Ping => send(serial, &Response::Pong),
        Request::Info => send(
            serial,
            &Response::Info(DeviceInfo {
                proto_version: PROTO_VERSION,
                heap_free: esp_alloc::HEAP.free() as u32,
                max_payload: MAX_PAYLOAD as u32,
                boot_count,
            }),
        ),
        Request::LoadExec {
            seq,
            entry_offset,
            arg,
            code,
        } => {
            if code.len() > MAX_PAYLOAD {
                send(serial, &Response::Error { seq, code: xt_runner_proto::ErrorCode::PayloadTooLarge });
                return;
            }
            if (entry_offset as usize) >= code.len() {
                send(serial, &Response::Error { seq, code: xt_runner_proto::ErrorCode::BadEntryOffset });
                return;
            }
            let result = run_payload(rtc, seq, &code, entry_offset, arg);
            send(serial, &Response::Ok { seq, result });
        }
    }
}

/// Copy `code` into an executable buffer and call it. If it faults or hangs,
/// the exception handler / watchdog resets the chip and the NEXT boot reports
/// the crash — so this function only returns on success.
fn run_payload(rtc: &mut Rtc<'_>, seq: u32, code: &[u8], entry_offset: u32, arg: u32) -> u32 {
    let buf = JitBuf::new(code);
    let entry = buf.exec_addr() + entry_offset as usize;

    ledger::arm(seq);
    rtc.rwdt
        .set_timeout(RwdtStage::Stage0, Duration::from_millis(PAYLOAD_WATCHDOG_MS));
    rtc.rwdt.enable();
    // enable() defaults stage 0 to ResetSystem, which ALSO resets the RTC
    // peripherals — wiping our RTC-RAM ledger, so a hang would look like a fresh
    // flash and no timeout would be reported. ResetCore leaves RTC RAM intact.
    rtc.rwdt
        .set_stage_action(RwdtStage::Stage0, RwdtStageAction::ResetCore);
    sync_code();

    // SAFETY: `entry` is the I-bus alias of freshly written, synced code holding
    // a complete windowed function `extern "C" fn(u32) -> u32`. A malformed
    // payload faults into `__user_exception` (→ reset) or hangs into the
    // watchdog (→ reset); either way control does not return here corrupted.
    let f: extern "C" fn(u32) -> u32 = unsafe { core::mem::transmute(entry) };
    let result = f(arg);

    rtc.rwdt.disable();
    ledger::disarm();
    result
}

fn send(serial: &mut UsbSerialJtag<'_, esp_hal::Blocking>, resp: &Response) {
    if let Ok(bytes) = xt_runner_proto::encode(resp) {
        // Leading COBS delimiter: on reset the ROM bootloader prints text to this
        // same FIFO, which would otherwise be prepended to our frame. The leading
        // 0x00 isolates that noise into its own (empty/garbage) frame the host
        // skips, so the real frame after it decodes cleanly.
        let _ = serial.write(&[0x00]);
        let _ = serial.write(&bytes);
        let _ = serial.flush_tx();
    }
}
