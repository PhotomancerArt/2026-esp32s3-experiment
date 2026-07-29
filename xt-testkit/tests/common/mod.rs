//! Shared bits for the per-board test files.

use lp_xt_inst::{encode, Inst, NullaryOp, Reg};

/// True when any of `vars` is set (non-empty) — the gate for a board's file.
pub fn configured(vars: &[&str]) -> bool {
    vars.iter()
        .any(|v| std::env::var(v).map(|p| !p.is_empty()).unwrap_or(false))
}

/// `entry a1,32; movi a2,42; retw` — returns 42 (encoder-built GV1 shape).
pub fn stub42_payload() -> Vec<u8> {
    let mut code = Vec::new();
    code.extend(encode(&Inst::Entry(Reg::new(1), 32)));
    code.extend(encode(&Inst::Movi(Reg::new(2), 42)));
    code.extend(encode(&Inst::Nullary(NullaryOp::Retw)));
    code
}

/// `entry a1,32; ill` — raises IllegalInstruction, forcing a crash + reset.
pub fn ill_payload() -> Vec<u8> {
    let mut code = Vec::new();
    code.extend(encode(&Inst::Entry(Reg::new(1), 32)));
    code.extend(encode(&Inst::Nullary(NullaryOp::Ill)));
    code
}
