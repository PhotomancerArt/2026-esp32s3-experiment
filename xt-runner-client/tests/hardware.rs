//! Hardware integration tests for xt-runner. Gated on `XT_DEVICE_PORT` — when
//! unset (no board), every test is skipped so `cargo test` stays green on CI.
//!
//! Flash the runner first: `cd fw/xt-runner && cargo run --release` (or espflash),
//! then: `XT_DEVICE_PORT=/dev/cu.usbmodem1101 cargo test -p xt-runner-client -- --nocapture`
//!
//! Payload bytes are objdump-derived (see the scratch payloads.s), never hand-recalled.

use xt_runner_client::{RunOutcome, Runner};
use xt_runner_proto::CrashKind;

/// GV1: `entry a1,32; movi a2,42; retw` — returns 42, ignores the arg.
const STUB42: &[u8] = &[0x36, 0x41, 0x00, 0x22, 0xa0, 0x2a, 0x90, 0x00, 0x00];
/// `entry a1,32; ill` — raises IllegalInstruction (EXCCAUSE 0).
const CRASH_ILL: &[u8] = &[0x36, 0x41, 0x00, 0x00, 0x00, 0x00];
/// `entry a1,32; j .` — infinite self-loop, caught by the watchdog.
const HANG: &[u8] = &[0x36, 0x41, 0x00, 0x06, 0xff, 0xff];

fn runner() -> Option<Runner> {
    match Runner::from_env() {
        None => {
            eprintln!("XT_DEVICE_PORT unset — skipping hardware test");
            None
        }
        Some(Ok(r)) => Some(r),
        Some(Err(e)) => panic!("failed to open device: {e}"),
    }
}

#[test]
fn ping_and_info() {
    let Some(mut r) = runner() else { return };
    r.ping().expect("ping");
    let info = r.info().expect("info");
    eprintln!("device info: {info:?}");
    assert_eq!(info.proto_version, xt_runner_proto::PROTO_VERSION);
    assert!(info.max_payload >= 1024);
}

#[test]
fn executes_good_payload() {
    let Some(mut r) = runner() else { return };
    let out = r.load_exec(1, STUB42.to_vec(), 0, 0).expect("load_exec");
    assert_eq!(out, RunOutcome::Ok(42), "stub42 must return 42");
}

#[test]
fn passes_argument_through() {
    // `entry a1,32; retw` — return a2, which is the callee's copy of arg (a10→a2).
    // Actually `retw` returns a2; with no movi, a2 holds the incoming arg.
    const IDENTITY: &[u8] = &[0x36, 0x41, 0x00, 0x90, 0x00, 0x00];
    let Some(mut r) = runner() else { return };
    let out = r.load_exec(2, IDENTITY.to_vec(), 0, 1234).expect("load_exec");
    assert_eq!(out, RunOutcome::Ok(1234), "identity must echo the arg");
}

#[test]
fn crash_is_reported_and_runner_recovers() {
    let Some(mut r) = runner() else { return };
    // A crashing payload resets the device; the client recovers the report.
    let out = r.load_exec(10, CRASH_ILL.to_vec(), 0, 0).expect("load_exec crash");
    match out {
        RunOutcome::Crash(report) => {
            eprintln!("crash report: {report:?}");
            assert_eq!(report.seq, 10);
            assert_eq!(report.kind, CrashKind::Exception);
        }
        other => panic!("expected crash, got {other:?}"),
    }
    // The runner must be alive again and able to run the next payload.
    let out = r.load_exec(11, STUB42.to_vec(), 0, 0).expect("load_exec after crash");
    assert_eq!(out, RunOutcome::Ok(42), "runner must recover after a crash");
}

#[test]
fn hang_is_caught_by_watchdog() {
    let Some(mut r) = runner() else { return };
    let out = r.load_exec(20, HANG.to_vec(), 0, 0).expect("load_exec hang");
    match out {
        RunOutcome::Crash(report) => {
            eprintln!("timeout report: {report:?}");
            assert_eq!(report.seq, 20);
            assert_eq!(report.kind, CrashKind::Timeout);
        }
        other => panic!("expected timeout, got {other:?}"),
    }
    let out = r.load_exec(21, STUB42.to_vec(), 0, 0).expect("load_exec after hang");
    assert_eq!(out, RunOutcome::Ok(42), "runner must recover after a hang");
}
