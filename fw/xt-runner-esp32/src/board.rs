//! Classic-ESP32 implementations of the `xt-runner-core` board traits: the
//! UART0 transport (through the board's USB-UART bridge) and the RWDT payload
//! watchdog. The code memory lives in `codemem`.

use esp_hal::rtc_cntl::{Rtc, RwdtStage, RwdtStageAction};
use esp_hal::time::Duration;
use esp_hal::uart::Uart;

use xt_runner_core::{PayloadWatchdog, Transport, PAYLOAD_WATCHDOG_MS};

/// The UART0 channel to the host, at 115200 8N1 (a real UART — unlike the
/// S3's USB-CDC, baud matters and the client must match).
///
/// Driving `esp_hal::uart::Uart` directly (constructed *after*
/// `esp_hal::init()`) programs the baud divisor for the current clock tree —
/// sidestepping the C1 gotcha where esp-println's raw FIFO writes go out at a
/// stale ROM divisor. esp-println is not linked into this crate at all: the
/// channel is pure binary.
pub struct UartTransport<'d> {
    uart: Uart<'d, esp_hal::Blocking>,
    /// Small drain buffer for `read_buffered` (the HW RX FIFO is 128 bytes;
    /// the protocol layer consumes one byte at a time).
    buf: [u8; 64],
    len: usize,
    pos: usize,
}

impl<'d> UartTransport<'d> {
    pub fn new(uart: Uart<'d, esp_hal::Blocking>) -> Self {
        UartTransport {
            uart,
            buf: [0; 64],
            len: 0,
            pos: 0,
        }
    }
}

impl Transport for UartTransport<'_> {
    fn read_byte(&mut self) -> Option<u8> {
        if self.pos < self.len {
            let b = self.buf[self.pos];
            self.pos += 1;
            return Some(b);
        }
        self.pos = 0;
        self.len = 0;
        match self.uart.read_buffered(&mut self.buf) {
            // Line errors (framing/parity/overflow) drop the read; the COBS
            // layer resynchronises on the next delimiter.
            Ok(0) | Err(_) => None,
            Ok(n) => {
                self.len = n;
                self.pos = 1;
                Some(self.buf[0])
            }
        }
    }

    fn write(&mut self, mut bytes: &[u8]) {
        // `Uart::write` queues at most a FIFO's worth per call.
        while !bytes.is_empty() {
            match self.uart.write(bytes) {
                Ok(n) => bytes = &bytes[n..],
                // TX error: drop the rest of the frame; the host skips the
                // resulting undecodable frame and times out / retries.
                Err(_) => return,
            }
        }
    }

    fn flush(&mut self) {
        // Blocks until the TX FIFO is empty AND the transmitter is idle, so a
        // response is fully on the wire before a payload can crash the chip.
        let _ = self.uart.flush();
    }
}

/// The RWDT armed around each payload call — same policy as the S3 runner.
pub struct RwdtWatchdog<'d>(pub Rtc<'d>);

impl PayloadWatchdog for RwdtWatchdog<'_> {
    fn arm(&mut self) {
        self.0
            .rwdt
            .set_timeout(RwdtStage::Stage0, Duration::from_millis(PAYLOAD_WATCHDOG_MS));
        self.0.rwdt.enable();
        // enable() defaults stage 0 to ResetSystem, which ALSO resets the RTC
        // peripherals — wiping our RTC-RAM ledger, so a hang would look like a
        // fresh flash and no timeout would be reported. ResetCore leaves RTC
        // RAM intact (C5 proved the classic ledger survives resets).
        self.0
            .rwdt
            .set_stage_action(RwdtStage::Stage0, RwdtStageAction::ResetCore);
    }

    fn disarm(&mut self) {
        self.0.rwdt.disable();
    }
}
