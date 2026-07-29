//! ESP32-S3 board-specific tests (P5): expectations that are genuinely
//! per-chip — the heap-backed code memory's capacity contract and crash
//! recovery over USB-CDC (the port drops and re-enumerates on reset) — live
//! here, in their own file, rather than as `if chip == ...` branches inside
//! shared tests. The shared corpus (`lp-xt-emu`/`xt-mini-emit` tests) covers
//! everything chip-common.
//!
//! Gated on the S3's own env vars (`XT_PORT_ESP32S3` / `XT_DEVICE_PORT`);
//! unset = skipped. Run single-threaded: the board is a shared resource.
//!
//! Payloads are encoder-built (`lp_xt_inst::encode`), never hand-encoded.

mod common;

use xt_runner_client::{Error, RunOutcome};
use xt_runner_proto::{Chip, ErrorCode, MAX_PAYLOAD};
use xt_testkit::Harness;

#[test]
fn esp32s3_board_contract() {
    if !common::configured(&["XT_PORT_ESP32S3", "XT_DEVICE_PORT"]) {
        eprintln!("XT_PORT_ESP32S3/XT_DEVICE_PORT unset — skipping ESP32-S3 board tests");
        return;
    }
    let mut h = Harness::from_env(20_000);
    let b = h
        .boards
        .iter_mut()
        .find(|b| b.chip() == Chip::Esp32S3)
        .expect("S3 configured, so discovery must have produced it");

    // --- capacity contract: heap-backed buffer, so the PROTOCOL cap binds
    // (the classic board derives the same number differently — see its file).
    assert_eq!(
        b.max_payload(),
        MAX_PAYLOAD,
        "S3 code memory is heap-backed: max_payload must be the protocol cap"
    );

    // --- capacity edge: one word past max_payload must be REFUSED with
    // PayloadTooLarge (not crash, not truncate), and the board stays alive.
    let oversize = vec![0u8; b.max_payload() + 4];
    let seq = b.next_seq();
    match b.board.runner.load_exec(seq, oversize, 0, 0) {
        Err(Error::DeviceError(ErrorCode::PayloadTooLarge)) => {}
        other => panic!("oversize payload must refuse with PayloadTooLarge, got {other:?}"),
    }
    b.board.runner.ping().expect("board must answer after refusing an oversize payload");

    // --- crash recovery over USB-CDC: a faulting payload resets the board,
    // the port drops and re-enumerates, and the client recovers the report.
    let before = b.board.runner.info().expect("info").boot_count;
    let ill = common::ill_payload();
    let seq = b.next_seq();
    match b.board.runner.load_exec(seq, ill, 0, 0).expect("load_exec") {
        RunOutcome::Crash(r) => {
            assert_eq!(r.seq, seq, "crash report must correlate by seq");
        }
        RunOutcome::Ok(v) => panic!("ILL payload returned Ok({v})"),
    }
    let after = b.board.runner.info().expect("info after crash").boot_count;
    assert!(
        after > before,
        "crash must reset the board (boot_count {before} -> {after})"
    );

    // --- and it still runs code correctly after the recovery round-trip.
    let stub = common::stub42_payload();
    let seq = b.next_seq();
    assert_eq!(
        b.board.runner.load_exec(seq, stub, 0, 0).expect("load_exec"),
        RunOutcome::Ok(42)
    );
}
