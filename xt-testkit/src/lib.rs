//! `xt-testkit` — the shared N-run test harness (P5).
//!
//! The corpus used to *dual*-run: emulator + "the board" (`XT_DEVICE_PORT`,
//! implicitly an S3). With two chips it **N-runs** through one code path:
//!
//! - the **emulator on every known [`BoardProfile`]** (so the classic map is
//!   exercised even boardless), and
//! - **every attached board**, each paired with the emulator run on *its own*
//!   profile — pairing the S3 silicon with the classic memory map (or vice
//!   versa) would silently compare different worlds.
//!
//! This is a crate (not a per-file `mod`) because the same harness serves test
//! suites in *two* crates — `xt-mini-emit/tests/*` and `lp-xt-emu/tests/*` —
//! and a shared module can only be shared within one crate. Both consume it as
//! a dev-dependency (the dev-dep cycle `lp-xt-emu -> xt-testkit -> lp-xt-emu`
//! is fine: dev-dependencies never enter the library build graph).
//!
//! ## Discovery semantics (deliberate, see the plan)
//!
//! - `XT_PORT_ESP32S3` / `XT_PORT_ESP32` name the boards; `XT_DEVICE_PORT` is
//!   a retained alias for the S3. **Unset var = board skipped** (emulator-only
//!   stays green boardless). **Configured-but-unreachable = panic** — silent
//!   skips hide regressions.
//! - The device's reported chip id is **verified** against the var it came
//!   from (ports renumber across replug; a mismatch is a loud error).
//!
//! ## Hardware serialization
//!
//! A *board* is the shared resource. Run hardware suites single-threaded
//! (`-- --test-threads=1`); as a second belt, [`Harness::from_env`] holds a
//! process-wide lock while any board is open, so parallel tests in one binary
//! serialize instead of fighting over the ports.
//!
//! ## Capacity
//!
//! Payload capacity differs per world (classic's code region is 92 KiB and
//! its firmware region-backed; the S3's buffer is heap-backed; the protocol
//! caps both at `MAX_PAYLOAD`). A case exceeding a world's capacity is
//! **skipped with a loud note naming the world and case** — never silently
//! truncated, never weakened.

use std::sync::{Mutex, MutexGuard};

use lp_xt_emu::emu::RunOutcome as EmuOutcome;
use lp_xt_emu::{BoardProfile, Emulator, TrapKind};
use xt_runner_client::{discover_boards, Board, RunOutcome as HwOutcome};
use xt_runner_proto::{Chip, CrashKind};

/// The board profiles every case emu-runs on, with the chip each models.
/// Order matters: the S3 comes first so `run_worlds()[0]` is the historical
/// dual-run emulator world (existing S3 expectations are unchanged).
pub fn known_profiles() -> [(Chip, BoardProfile); 2] {
    [
        (Chip::Esp32S3, BoardProfile::esp32s3()),
        (Chip::Esp32, BoardProfile::esp32()),
    ]
}

/// The emulator profile matching a chip — the pairing `run_worlds` uses for
/// hardware diffs.
pub fn profile_for(chip: Chip) -> BoardProfile {
    match chip {
        Chip::Esp32S3 => BoardProfile::esp32s3(),
        Chip::Esp32 => BoardProfile::esp32(),
    }
}

/// An attached board plus its emulator profile and a per-board seq counter.
pub struct TestBoard {
    pub board: Board,
    pub profile: BoardProfile,
    seq: u32,
}

impl TestBoard {
    pub fn chip(&self) -> Chip {
        self.board.chip
    }

    /// World label, e.g. `hw:esp32@/dev/cu.usbserial-1440`.
    pub fn world(&self) -> String {
        format!("hw:{}", self.board.label())
    }

    /// Next payload seq for this board (monotonic; crash reports correlate on it).
    pub fn next_seq(&mut self) -> u32 {
        self.seq += 1;
        self.seq
    }

    /// The largest payload this board accepts, as the device itself reports
    /// (min of the protocol cap and the firmware's `CodeMem::capacity()`).
    pub fn max_payload(&self) -> usize {
        self.board.info.max_payload as usize
    }
}

/// Known-answer expectation for a case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Expect {
    Ok(u32),
    /// The case must crash with this class in every world (hardware crash
    /// kinds are normalized: `Timeout` -> `Timeout`, everything else ->
    /// `Exception`, as the device cannot distinguish further).
    Crash(TrapKind),
}

/// One world's normalized outcome.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Ok(u32),
    /// Crash class + EXCCAUSE (0 where not applicable).
    Crash { kind: TrapKind, cause: u32 },
}

/// One world's result: which world, and what happened.
#[derive(Clone, Debug)]
pub struct WorldResult {
    /// `emu:<chip>` or `hw:<chip>@<port>`.
    pub world: String,
    pub chip: Chip,
    pub hardware: bool,
    pub outcome: Outcome,
}

impl WorldResult {
    /// The Ok value, panicking (with the case name) on a crash.
    pub fn ok(&self, name: &str) -> u32 {
        match self.outcome {
            Outcome::Ok(v) => v,
            Outcome::Crash { kind, cause } => panic!(
                "[{name}] {} crashed: {kind:?} cause={cause}",
                self.world
            ),
        }
    }
}

/// Process-wide hardware lock (see module docs). Poisoning is ignored: a
/// panicking test already failed; later tests should still reach the boards.
static HW_LOCK: Mutex<()> = Mutex::new(());

fn lock_hw() -> MutexGuard<'static, ()> {
    HW_LOCK.lock().unwrap_or_else(|p| p.into_inner())
}

/// The N-run harness: every known emulator profile + every attached board.
pub struct Harness {
    pub boards: Vec<TestBoard>,
    _hw_guard: Option<MutexGuard<'static, ()>>,
}

impl Harness {
    /// Discover the configured boards. `seq_base` seeds each board's payload
    /// seq counter (per-file bases keep crash reports attributable, as the
    /// old per-test counters did).
    ///
    /// Panics — loudly, by design — when a configured board is unreachable,
    /// reports the wrong chip, or speaks the wrong protocol version.
    pub fn from_env(seq_base: u32) -> Harness {
        let configured = xt_runner_client::BOARD_ENV
            .iter()
            .flat_map(|(_, vars, _)| vars.iter())
            .any(|v| std::env::var(v).map(|p| !p.is_empty()).unwrap_or(false));
        let guard = configured.then(lock_hw);
        let boards = discover_boards()
            .unwrap_or_else(|e| panic!("board discovery FAILED (configured board must work): {e}"));
        if boards.is_empty() {
            eprintln!(
                "no board env vars set (XT_PORT_ESP32S3 / XT_PORT_ESP32 / XT_DEVICE_PORT) — \
                 emulator-only"
            );
        }
        let boards = boards
            .into_iter()
            .map(|board| {
                eprintln!(
                    "board {} ({}): max_payload={} heap_free={} boot_count={}",
                    board.label(),
                    board.env_var,
                    board.info.max_payload,
                    board.info.heap_free,
                    board.info.boot_count
                );
                TestBoard {
                    profile: profile_for(board.chip),
                    board,
                    seq: seq_base,
                }
            })
            .collect();
        Harness {
            boards,
            _hw_guard: guard,
        }
    }

    pub fn has_hardware(&self) -> bool {
        !self.boards.is_empty()
    }

    /// Visit every attached board (for board-level tests: info checks,
    /// capacity edges, transport behavior).
    pub fn for_each_board(&mut self, mut f: impl FnMut(&mut TestBoard)) {
        for b in &mut self.boards {
            f(b);
        }
    }

    /// The N-run primitive: run `code` on the emulator under **every** known
    /// profile, then on **every** attached board, and return one normalized
    /// [`WorldResult`] per world. Transport failures panic. A payload
    /// exceeding a world's capacity skips that world with a loud note (never
    /// truncated, never weakened).
    pub fn run_worlds(
        &mut self,
        name: &str,
        code: &[u8],
        entry_offset: u32,
        arg: u32,
    ) -> Vec<WorldResult> {
        let mut out = Vec::new();
        for (chip, profile) in known_profiles() {
            if code.len() > profile.code_region_len {
                eprintln!(
                    "SKIP-CAPACITY world=emu:{chip} case={name}: payload {} B exceeds the \
                     {chip} code region ({} B) — case NOT run on this world",
                    code.len(),
                    profile.code_region_len
                );
                continue;
            }
            let mut emu = Emulator::with_profile(profile);
            let outcome = match emu.run(code, entry_offset, arg) {
                EmuOutcome::Ok(v) => Outcome::Ok(v),
                EmuOutcome::Trap(t) => Outcome::Crash {
                    kind: t.kind,
                    cause: t.cause,
                },
            };
            out.push(WorldResult {
                world: format!("emu:{chip}"),
                chip,
                hardware: false,
                outcome,
            });
        }
        for b in &mut self.boards {
            let world = b.world();
            if code.len() > b.max_payload() {
                eprintln!(
                    "SKIP-CAPACITY world={world} case={name}: payload {} B exceeds the board's \
                     max_payload ({} B) — case NOT run on this board",
                    code.len(),
                    b.max_payload()
                );
                continue;
            }
            let seq = b.next_seq();
            let hw = b
                .board
                .runner
                .load_exec(seq, code.to_vec(), entry_offset, arg)
                .unwrap_or_else(|e| panic!("[{name}] {world}: load_exec failed: {e}"));
            let outcome = match hw {
                HwOutcome::Ok(v) => Outcome::Ok(v),
                HwOutcome::Crash(r) => Outcome::Crash {
                    kind: match r.kind {
                        CrashKind::Timeout => TrapKind::Timeout,
                        _ => TrapKind::Exception,
                    },
                    cause: r.cause,
                },
            };
            out.push(WorldResult {
                world,
                chip: b.chip(),
                hardware: true,
                outcome,
            });
        }
        out
    }

    /// Every world must return `Ok(expect)` — the workhorse for
    /// known-answer, position-independent cases.
    pub fn nrun(&mut self, name: &str, code: &[u8], entry_offset: u32, arg: u32, expect: u32) {
        self.nrun_expect(name, code, entry_offset, arg, Expect::Ok(expect));
    }

    /// Every world must match `expect` (value, or crash class). For crash
    /// cases the *EXCCAUSE* must also agree across every world that reports
    /// one as an `Exception` (the emulator models causes; hardware reports
    /// EXCCAUSE verbatim).
    pub fn nrun_expect(
        &mut self,
        name: &str,
        code: &[u8],
        entry_offset: u32,
        arg: u32,
        expect: Expect,
    ) {
        let results = self.run_worlds(name, code, entry_offset, arg);
        let mut exc_cause: Option<(String, u32)> = None;
        for w in &results {
            match (expect, w.outcome) {
                (Expect::Ok(want), Outcome::Ok(got)) => {
                    assert_eq!(
                        got, want,
                        "[{name}] {} result mismatch (arg={arg})",
                        w.world
                    );
                }
                (Expect::Crash(kind), Outcome::Crash { kind: got, cause }) => {
                    assert_eq!(
                        got, kind,
                        "[{name}] {} crash-class mismatch (arg={arg}, cause={cause})",
                        w.world
                    );
                    if kind == TrapKind::Exception {
                        match &exc_cause {
                            None => exc_cause = Some((w.world.clone(), cause)),
                            Some((w0, c0)) => assert_eq!(
                                cause, *c0,
                                "[{name}] EXCCAUSE diff: {}={cause} vs {w0}={c0}",
                                w.world
                            ),
                        }
                    }
                }
                (want, got) => panic!(
                    "[{name}] {} outcome {got:?} != expected {want:?} (arg={arg})",
                    w.world
                ),
            }
        }
    }

    /// Position-independent measurement: every world must agree on the value;
    /// returns it. (The measurement itself is the caller's finding.)
    pub fn measure(&mut self, name: &str, code: &[u8], entry_offset: u32, arg: u32) -> u32 {
        let results = self.run_worlds(name, code, entry_offset, arg);
        let mut agreed: Option<(String, u32)> = None;
        for w in &results {
            let v = w.ok(name);
            match &agreed {
                None => agreed = Some((w.world.clone(), v)),
                Some((w0, v0)) => assert_eq!(
                    v, *v0,
                    "[{name}] {} vs {w0} measurement diff (arg={arg})",
                    w.world
                ),
            }
        }
        agreed
            .unwrap_or_else(|| panic!("[{name}] no world ran the case (all capacity-skipped?)"))
            .1
    }

    /// Run everywhere WITHOUT cross-world value assertions — for programs
    /// whose raw value is position-dependent (mangled return addresses, SPs)
    /// where only a *predicate* of the value is comparable. Crashes panic.
    /// Results come back per world; `[0]` is `emu:esp32s3`, the historical
    /// dual-run emulator world.
    pub fn run_all(
        &mut self,
        name: &str,
        code: &[u8],
        entry_offset: u32,
        arg: u32,
    ) -> Vec<(String, u32)> {
        self.run_worlds(name, code, entry_offset, arg)
            .into_iter()
            .map(|w| {
                let v = w.ok(name);
                (w.world, v)
            })
            .collect()
    }
}
