//! Host client for `xt-runner`: send Xtensa code payloads to the resident
//! ESP32-S3 firmware over USB-Serial-JTAG and get back results or crash reports.
//!
//! The tricky part is crash recovery: when a payload faults or hangs, the device
//! resets, which drops the USB-CDC port (it re-enumerates under the same name).
//! [`Runner::load_exec`] handles that — it reopens the port and reads the
//! unsolicited `CrashReport` the firmware emits on its next boot.

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

    /// Open using the `XT_DEVICE_PORT` env var; returns `None` if unset (so
    /// tests can skip hardware cleanly).
    pub fn from_env() -> Option<Result<Runner, Error>> {
        std::env::var("XT_DEVICE_PORT").ok().map(|p| Runner::open(&p))
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

        // Good path: a prompt Ok. Crash path: the port drops, the device
        // reboots (~1-2s), and a Crash frame arrives on the reopened port.
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
    // USB-Serial-JTAG ignores baud; the value is a placeholder.
    serialport::new(path, 115_200)
        .timeout(Duration::from_millis(100))
        .open()
}
