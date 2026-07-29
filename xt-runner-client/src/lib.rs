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

use xt_runner_proto::{Chip, CrashReport, DeviceInfo, Request, Response, PROTO_VERSION};

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
    /// A board was *configured* (its env var is set) but could not be opened
    /// or did not answer the handshake. This is a hard error by design: a
    /// configured board silently skipping would hide regressions.
    #[error("{env_var}={port}: configured board unreachable: {source}")]
    Unreachable {
        env_var: &'static str,
        port: String,
        #[source]
        source: Box<Error>,
    },
    /// The device on a configured port reports a different chip than the env
    /// var claims. Loud by design: serial port numbering is NOT stable across
    /// replug order (the S3 once moved usbmodem1101 -> usbmodem1301 when a
    /// third board appeared), so a stale var would silently test the wrong
    /// board. Fix the env var; never trust the port number.
    #[error(
        "{env_var}={port}: device reports chip '{reported}', expected '{expected}' — \
         WRONG BOARD on this port (port names renumber across replug; identify by chip, \
         fix the env var)"
    )]
    ChipMismatch {
        env_var: &'static str,
        port: String,
        expected: Chip,
        reported: Chip,
    },
    /// `XT_DEVICE_PORT` (the historical S3 alias) and `XT_PORT_ESP32S3` are
    /// both set but name different ports — ambiguous, refuse to guess.
    #[error(
        "XT_PORT_ESP32S3={primary} and XT_DEVICE_PORT={alias} disagree; \
         set one (or both to the same port)"
    )]
    AliasConflict { primary: String, alias: String },
    /// Firmware speaks a different protocol version — reflash the runner.
    #[error(
        "{env_var}={port}: device speaks proto v{got}, host expects v{want} — \
         reflash the runner firmware"
    )]
    ProtoMismatch {
        env_var: &'static str,
        port: String,
        got: u32,
        want: u32,
    },
}

/// Outcome of running a payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunOutcome {
    Ok(u32),
    Crash(CrashReport),
}

pub struct Runner {
    port_path: String,
    baud: u32,
    port: Box<dyn serialport::SerialPort>,
    /// Leftover bytes read past a frame delimiter.
    rx: Vec<u8>,
}

/// Per-board configuration: chip, the env vars naming its port (first entry is
/// the primary; later entries are historical aliases), and the baud rate. Baud
/// is carried per board because a UART bridge (classic ESP32, fixed 115200 8N1
/// in the firmware) genuinely needs it, while USB-CDC (S3) ignores it.
pub const BOARD_ENV: [(Chip, &[&str], u32); 2] = [
    (Chip::Esp32S3, &["XT_PORT_ESP32S3", "XT_DEVICE_PORT"], 115_200),
    (Chip::Esp32, &["XT_PORT_ESP32"], 115_200),
];

/// A discovered, verified, open board.
pub struct Board {
    /// Which chip this is — verified against the device's own report, not
    /// assumed from the env var.
    pub chip: Chip,
    /// The env var that configured this board (diagnostics).
    pub env_var: &'static str,
    pub port: String,
    /// The device's `Info` reply from discovery (`info.chip == chip` holds).
    pub info: DeviceInfo,
    pub runner: Runner,
}

impl Board {
    /// Diagnostic label, e.g. `esp32@/dev/cu.usbserial-1440`.
    pub fn label(&self) -> String {
        format!("{}@{}", self.chip, self.port)
    }
}

/// Enumerate the configured boards from the per-board env vars.
///
/// Semantics (P5, deliberate):
/// - **Unset var = that board is skipped** (returned Vec simply omits it; an
///   empty Vec means emulator-only).
/// - **Configured-but-unreachable = `Err`** — never a silent skip.
/// - Each device's reported chip id is verified against the env var it came
///   from; a mismatch is [`Error::ChipMismatch`], never a silent swap (port
///   names renumber across replug order — identify boards by chip, not port).
pub fn discover_boards() -> Result<Vec<Board>, Error> {
    let mut boards = Vec::new();
    for (chip, vars, baud) in BOARD_ENV {
        let set: Vec<(&'static str, String)> = vars
            .iter()
            .filter_map(|v| match std::env::var(v) {
                Ok(p) if !p.is_empty() => Some((*v, p)),
                _ => None,
            })
            .collect();
        if set.len() > 1 && set.iter().any(|(_, p)| *p != set[0].1) {
            return Err(Error::AliasConflict {
                primary: set[0].1.clone(),
                alias: set[1].1.clone(),
            });
        }
        let Some((env_var, port)) = set.into_iter().next() else {
            continue; // unset: this board is not part of the run
        };
        boards.push(open_board(chip, env_var, port, baud)?);
    }
    Ok(boards)
}

/// Open + handshake + verify one configured board.
fn open_board(chip: Chip, env_var: &'static str, port: String, baud: u32) -> Result<Board, Error> {
    let unreachable = |source: Error| Error::Unreachable {
        env_var,
        port: port.clone(),
        source: Box::new(source),
    };
    let mut runner = Runner::open_with_baud(&port, baud).map_err(&unreachable)?;
    // Opening a UART-bridge port can itself reset the board (DTR/RTS
    // auto-reset wiring), so the first requests may land mid-reboot: retry the
    // handshake until the runner answers or the deadline passes.
    let deadline = Instant::now() + Duration::from_secs(8);
    let info = loop {
        match runner.info() {
            Ok(i) => break i,
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(unreachable(e));
                }
            }
        }
    };
    if info.proto_version != PROTO_VERSION {
        return Err(Error::ProtoMismatch {
            env_var,
            port,
            got: info.proto_version,
            want: PROTO_VERSION,
        });
    }
    if info.chip != chip {
        return Err(Error::ChipMismatch {
            env_var,
            port,
            expected: chip,
            reported: info.chip,
        });
    }
    Ok(Board {
        chip,
        env_var,
        port,
        info,
        runner,
    })
}

impl Runner {
    /// Open the runner on `port_path` (e.g. `/dev/cu.usbmodem1101`) at the
    /// default 115200 baud.
    pub fn open(port_path: &str) -> Result<Runner, Error> {
        Runner::open_with_baud(port_path, 115_200)
    }

    /// Open at an explicit baud rate (per-board setting: a UART bridge needs
    /// it, USB-CDC ignores it).
    pub fn open_with_baud(port_path: &str, baud: u32) -> Result<Runner, Error> {
        let port = open_port(port_path, baud)?;
        Ok(Runner {
            port_path: port_path.to_string(),
            baud,
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
            if let Ok(port) = open_port(&self.port_path, self.baud) {
                self.port = port;
                return Ok(());
            }
        }
    }
}

fn open_port(path: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>, serialport::Error> {
    // 115200 is load-bearing on the classic ESP32's UART bridge (the firmware
    // fixes UART0 at 115200 8N1); the S3's USB-Serial-JTAG ignores it.
    // NOTE: opening a UART-bridge port can itself reset the board (DTR/RTS
    // auto-reset wiring) — callers must not assume the device kept state
    // across an open.
    serialport::new(path, baud)
        .timeout(Duration::from_millis(100))
        .open()
}
