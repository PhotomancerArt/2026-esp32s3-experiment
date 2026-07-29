//! ESP32-S3 implementations of the `xt-runner-core` board traits: the
//! USB-Serial-JTAG transport, the heap-alias code memory, and the RWDT payload
//! watchdog.

use esp_hal::rtc_cntl::{Rtc, RwdtStage, RwdtStageAction};
use esp_hal::time::Duration;
use esp_hal::usb_serial_jtag::UsbSerialJtag;

use xt_runner_core::{CodeMem, LoadError, PayloadWatchdog, Transport, PAYLOAD_WATCHDOG_MS};
use xt_runner_proto::MAX_PAYLOAD;

use crate::jitbuf::{sync_code, JitBuf};

/// The USB-Serial-JTAG channel to the host.
pub struct SerialTransport<'d>(pub UsbSerialJtag<'d, esp_hal::Blocking>);

impl Transport for SerialTransport<'_> {
    fn read_byte(&mut self) -> Option<u8> {
        // UsbSerialJtag's read error type is `Infallible`: the only `Err` is
        // `WouldBlock`, so `Option` loses nothing.
        self.0.read_byte().ok()
    }

    fn write(&mut self, bytes: &[u8]) {
        let _ = self.0.write(bytes);
    }

    fn flush(&mut self) {
        let _ = self.0.flush_tx();
    }
}

/// S3 code memory: a fresh heap buffer per payload, executed through the
/// uniform SRAM1 I-bus alias (see `jitbuf`). The buffer lives until `release`
/// so the entry address stays valid across the call.
pub struct HeapAliasCodeMem {
    buf: Option<JitBuf>,
}

impl HeapAliasCodeMem {
    pub fn new() -> Self {
        HeapAliasCodeMem { buf: None }
    }
}

impl CodeMem for HeapAliasCodeMem {
    fn load(&mut self, code: &[u8]) -> Result<usize, LoadError> {
        let buf = JitBuf::new(code);
        let exec = buf.exec_addr();
        self.buf = Some(buf);
        Ok(exec)
    }

    fn sync(&mut self) {
        sync_code();
    }

    fn release(&mut self) {
        // Free the payload buffer immediately (as the original runner did by
        // scope), so `Info`'s heap_free never counts a stale payload.
        self.buf = None;
    }

    fn capacity(&self) -> usize {
        // Heap-backed, so the protocol cap is the binding limit on this board.
        MAX_PAYLOAD
    }
}

/// The RWDT armed around each payload call.
pub struct RwdtWatchdog<'d>(pub Rtc<'d>);

impl PayloadWatchdog for RwdtWatchdog<'_> {
    fn arm(&mut self) {
        self.0
            .rwdt
            .set_timeout(RwdtStage::Stage0, Duration::from_millis(PAYLOAD_WATCHDOG_MS));
        self.0.rwdt.enable();
        // enable() defaults stage 0 to ResetSystem, which ALSO resets the RTC
        // peripherals — wiping our RTC-RAM ledger, so a hang would look like a
        // fresh flash and no timeout would be reported. ResetCore leaves RTC RAM
        // intact.
        self.0
            .rwdt
            .set_stage_action(RwdtStage::Stage0, RwdtStageAction::ResetCore);
    }

    fn disarm(&mut self) {
        self.0.rwdt.disable();
    }
}
