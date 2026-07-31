//! M6-P1 FP capability probe driver (lp2025 f32 roadmap).
//!
//! Replays the 26 assembler-derived probe payloads from lp2025's
//! `lp-xt/fixtures/fp/probe.S` (branch `claude/f32-m6-p1-xt-inst-fp`,
//! extracted by `probes.sh`) against a resident `xt-runner-esp32s3` and prints
//! one verdict per probe. Verdict rules come from probe.S's own header:
//!
//! - returns its staged id      -> PRESENT (and CP0 armed)
//! - Exception, EXCCAUSE 32     -> present, CPENABLE not armed (Coprocessor0Disabled)
//! - Exception, EXCCAUSE 0      -> ABSENT (IllegalInstruction)
//! - anything else              -> UNEXPECTED, verbatim for the session log
//!
//! Run: XT_PORT_ESP32S3=/dev/cu.usbmodemXXXX cargo run -p xt-runner-client --example fp_probe

use xt_runner_client::{discover_boards, RunOutcome};

include!("fp_probes_table.rs");

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut boards = discover_boards()?;
    let board = boards
        .iter_mut()
        .find(|b| format!("{:?}", b.chip).contains("S3"))
        .ok_or("no ESP32-S3 configured (set XT_PORT_ESP32S3)")?;
    let runner = &mut board.runner;
    runner.ping()?;
    let info = runner.info()?;
    println!(
        "# runner: chip={:?} proto={} heap_free={} boot_count={}",
        info.chip, info.proto_version, info.heap_free, info.boot_count
    );
    println!("# {} probes\n", FP_PROBES.len());
    println!("| probe | outcome | verdict |");
    println!("|---|---|---|");

    let filter: Vec<String> = std::env::args().skip(1).collect();
    for (i, (name, code)) in FP_PROBES
        .iter()
        .filter(|(n, _)| filter.is_empty() || filter.iter().any(|f| f == n))
        .enumerate()
    {
        let seq = 1000 + i as u32;
        let outcome = runner.load_exec(seq, code.to_vec(), 0, 0)?;
        let (raw, verdict) = match outcome {
            RunOutcome::Ok(v) => (format!("Ok({v})"), "PRESENT".to_string()),
            RunOutcome::Crash(ref r) => {
                let v = match r.cause {
                    32 => "present, CPENABLE NOT ARMED".to_string(),
                    0 => "ABSENT (illegal instruction)".to_string(),
                    c => format!("UNEXPECTED EXCCAUSE {c}"),
                };
                (format!("{:?}", r), v)
            }
        };
        println!("| {name} | {raw} | {verdict} |");
    }
    Ok(())
}
