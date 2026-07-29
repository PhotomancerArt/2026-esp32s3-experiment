//! Host client for `xt-runner`: send Xtensa code payloads to a resident
//! runner firmware (ESP32-S3 over USB-Serial-JTAG, classic ESP32 over its
//! USB-UART bridge at 115200) and get back results or crash reports.
//! `DeviceInfo::chip` says which board a port actually talks to.
//!
//! The tricky part is crash recovery: when a payload faults or hangs, the
//! device resets. What the host sees depends on the transport, and
//! [`Runner::load_exec`] tolerates both:
//!
//! - **USB-CDC (S3)**: the port drops and re-enumerates under the same name —
//!   reads error, so the client reopens the port and then reads the
//!   unsolicited `CrashReport` the firmware emits on its next boot.
//! - **UART bridge (classic)**: the port does NOT drop (FINDINGS C5) — reads
//!   just go quiet during the reboot, then ROM boot noise arrives (skipped as
//!   undecodable frames) followed by the `CrashReport`. No reopen happens.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use xt_runner_proto::{CrashReport, DeviceInfo, Request, Response};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("serial: {0}")]
    Serial(#[from] serialport::Error),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("protocol decode: {0}")]
    Decode(postcard::Error),
    #[error("timed out waiting for device")]
    Timeout,
    #[error("unexpected response: {0:?}")]
    Unexpected(Response),
    #[error("device reported error: {0:?}")]
    DeviceError(xt_runner_proto::ErrorCode),
}

/// Outcome of running a payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunOutcome {
    Ok(u32),
    Crash(CrashReport),
}

pub struct Runner {
    port_path: String,
    port: Box<dyn serialport::SerialPort>,
    /// Leftover bytes read past a frame delimiter.
    rx: Vec<u8>,
}

impl Runner {
    /// Open the runner on `port_path` (e.g. `/dev/cu.usbmodem1101`).
    pub fn open(port_path: &str) -> Result<Runner, Error> {
        let port = open_port(port_path)?;
        Ok(Runner {
            port_path: port_path.to_string(),
            port,
            rx: Vec::new(),
        })
    }

    /// Open using the first set of `XT_DEVICE_PORT` (the S3 alias),
    /// `XT_PORT_ESP32S3`, `XT_PORT_ESP32`; returns `None` if all are unset
    /// (so tests can skip hardware cleanly). P5's N-run harness drives the
    /// per-board vars individually; this helper just picks *a* board.
    pub fn from_env() -> Option<Result<Runner, Error>> {
        ["XT_DEVICE_PORT", "XT_PORT_ESP32S3", "XT_PORT_ESP32"]
            .iter()
            .find_map(|v| std::env::var(v).ok())
            .map(|p| Runner::open(&p))
    }

    pub fn ping(&mut self) -> Result<(), Error> {
        self.send(&Request::Ping)?;
        match self.recv_frame(Duration::from_secs(2))? {
            Response::Pong => Ok(()),
            other => Err(Error::Unexpected(other)),
        }
    }

    pub fn info(&mut self) -> Result<DeviceInfo, Error> {
        self.send(&Request::Info)?;
        match self.recv_frame(Duration::from_secs(2))? {
            Response::Info(i) => Ok(i),
            other => Err(Error::Unexpected(other)),
        }
    }

    /// Load `code` into an executable buffer and call
    /// `(code + entry_offset)(arg)`. Returns the result, or a `CrashReport` if
    /// the payload faulted/hung (recovered across the device's auto-reset).
    pub fn load_exec(
        &mut self,
        seq: u32,
        code: Vec<u8>,
        entry_offset: u32,
        arg: u32,
    ) -> Result<RunOutcome, Error> {
        self.send(&Request::LoadExec {
            seq,
            entry_offset,
            arg,
            code,
        })?;

        // Good path: a prompt Ok. Crash path: the device reboots (~1-2s) and
        // an unsolicited Crash frame arrives — over USB-CDC the port drops
        // first (reads error → reopen below); over a UART bridge the port
        // stays open and the frame simply shows up after the boot noise.
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match self.recv_frame_until(deadline) {
                Ok(Response::Ok { seq: s, result }) if s == seq => {
                    return Ok(RunOutcome::Ok(result))
                }
                Ok(Response::Crash(report)) if report.seq == seq => {
                    return Ok(RunOutcome::Crash(report))
                }
                Ok(Response::Error { seq: s, code }) if s == seq => {
                    return Err(Error::DeviceError(code))
                }
                // Stale/unrelated frame (e.g. a crash from an earlier seq) —
                // keep waiting.
                Ok(_) => continue,
                Err(Error::Io(_)) | Err(Error::Serial(_)) => {
                    // Device reset dropped the port; wait for it to re-enumerate.
                    if Instant::now() >= deadline {
                        return Err(Error::Timeout);
                    }
                    self.reopen(deadline)?;
                }
                Err(Error::Timeout) => return Err(Error::Timeout),
                Err(e) => return Err(e),
            }
        }
    }

    fn send(&mut self, req: &Request) -> Result<(), Error> {
        let bytes = xt_runner_proto::encode(req).map_err(Error::Decode)?;
        self.port.write_all(&bytes)?;
        self.port.flush()?;
        Ok(())
    }

    fn recv_frame(&mut self, timeout: Duration) -> Result<Response, Error> {
        self.recv_frame_until(Instant::now() + timeout)
    }

    /// Read bytes until a COBS delimiter (0x00), then decode one frame. Empty
    /// frames (stray delimiters) and undecodable frames (ROM boot-log noise that
    /// precedes the real frame after a device reset) are skipped, not surfaced —
    /// the caller only ever sees a well-formed `Response` or a timeout.
    fn recv_frame_until(&mut self, deadline: Instant) -> Result<Response, Error> {
        let mut byte = [0u8; 1];
        loop {
            if let Some(pos) = self.rx.iter().position(|&b| b == 0) {
                let mut frame: Vec<u8> = self.rx.drain(..=pos).collect();
                if frame.len() <= 1 {
                    continue; // empty frame (stray / leading delimiter)
                }
                match xt_runner_proto::decode::<Response>(&mut frame) {
                    Ok(resp) => return Ok(resp),
                    Err(_) => continue, // garbage frame (boot noise) — keep reading
                }
            }
            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }
            match self.port.read(&mut byte) {
                Ok(0) => continue,
                Ok(_) => self.rx.push(byte[0]),
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }
    }

    /// Reopen the port after a device reset, retrying until it re-enumerates.
    fn reopen(&mut self, deadline: Instant) -> Result<(), Error> {
        self.rx.clear();
        loop {
            std::thread::sleep(Duration::from_millis(200));
            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }
            if let Ok(port) = open_port(&self.port_path) {
                self.port = port;
                return Ok(());
            }
        }
    }
}

fn open_port(path: &str) -> Result<Box<dyn serialport::SerialPort>, serialport::Error> {
    // 115200 is load-bearing on the classic ESP32's UART bridge (the firmware
    // fixes UART0 at 115200 8N1); the S3's USB-Serial-JTAG ignores it.
    // NOTE: opening a UART-bridge port can itself reset the board (DTR/RTS
    // auto-reset wiring) — callers must not assume the device kept state
    // across an open.
    serialport::new(path, 115_200)
        .timeout(Duration::from_millis(100))
        .open()
}
