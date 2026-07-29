//! Classic-ESP32 (LX6) board-specific tests (P5): expectations that are
//! genuinely per-chip — the fixed SRAM1 code region's capacity contract and
//! crash recovery over the UART bridge (the port does NOT drop across a
//! reset, FINDINGS C5) — live here, in their own file, rather than as
//! `if chip == ...` branches inside shared tests. The word-mirror memory
//! model itself is pinned emulator-side in `lp-xt-emu/tests/board_profile.rs`
//! and firmware-side by `fw/xt-runner-esp32`'s const asserts; the shared
//! corpus exercises it end to end.
//!
//! Gated on `XT_PORT_ESP32`; unset = skipped. Run single-threaded: the board
//! is a shared resource.
//!
//! Payloads are encoder-built (`lp_xt_inst::encode`), never hand-encoded.

mod common;

use xt_runner_client::{Error, RunOutcome};
use xt_runner_proto::{Chip, ErrorCode, MAX_PAYLOAD};
use xt_testkit::Harness;

/// The classic runner's fixed SRAM1 code region (92 KiB), mirrored from
/// `fw/xt-runner-esp32/src/codemem.rs` and `BoardProfile::esp32()`.
const CLASSIC_CODE_REGION_LEN: usize = 0x0001_7000;

#[test]
fn esp32_board_contract() {
    if !common::configured(&["XT_PORT_ESP32"]) {
        eprintln!("XT_PORT_ESP32 unset — skipping classic-ESP32 board tests");
        return;
    }
    let mut h = Harness::from_env(21_000);
    let b = h
        .boards
        .iter_mut()
        .find(|b| b.chip() == Chip::Esp32)
        .expect("classic board configured, so discovery must have produced it");

    // --- capacity contract: a FIXED 92 KiB SRAM1 region (the classic heap is
    // not executable), so max_payload = min(protocol cap, region). The region
    // must match the emulator profile, or emu-vs-hw capacity skips diverge.
    assert_eq!(b.profile.code_region_len, CLASSIC_CODE_REGION_LEN);
    assert_eq!(
        b.max_payload(),
        MAX_PAYLOAD.min(CLASSIC_CODE_REGION_LEN),
        "classic max_payload must be min(protocol cap, SRAM1 code region)"
    );
    assert!(
        b.max_payload() <= b.profile.code_region_len,
        "board must never accept a payload its emulator profile cannot hold"
    );

    // --- request-error path: a bad entry offset must be REFUSED (not run,
    // not crash), and the board must stay alive — probed with a tiny payload.
    //
    // NOTE (P5 finding, measured 2026-07-28): the S3's true oversize probe
    // (max_payload + 4 bytes, see board_esp32s3.rs) CANNOT run on classic —
    // receiving a ~32.9 KB frame needs ~3x that transiently (rx accumulator +
    // handle_frame's copy + the decoded Vec ≈ 99 KB) and the classic runner
    // has only ~97 KB heap free, so the frame OOM-panics the firmware and
    // resets the board (observed: boot_count incremented, host timed out)
    // instead of eliciting PayloadTooLarge. Until the firmware RX path stops
    // buffering three copies, PayloadTooLarge is unreachable over this
    // transport; do NOT "fix" this by sending the frame anyway.
    let stub = common::stub42_payload();
    let bad_entry = stub.len() as u32 + 4;
    let seq = b.next_seq();
    match b.board.runner.load_exec(seq, stub.clone(), bad_entry, 0) {
        Err(Error::DeviceError(ErrorCode::BadEntryOffset)) => {}
        other => panic!("out-of-range entry_offset must refuse with BadEntryOffset, got {other:?}"),
    }
    b.board.runner.ping().expect("board must answer after refusing a bad entry offset");

    // --- crash recovery over the UART bridge: a faulting payload resets the
    // board; the PORT STAYS OPEN (no re-enumeration, FINDINGS C5) — reads go
    // quiet, ROM boot noise arrives as garbage frames, then the crash report.
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

    // --- and it still runs code correctly after the recovery round-trip
    // (the reload also re-walks the word-mirrored D-bus write path).
    let seq = b.next_seq();
    assert_eq!(
        b.board.runner.load_exec(seq, stub, 0, 0).expect("load_exec"),
        RunOutcome::Ok(42)
    );
}
