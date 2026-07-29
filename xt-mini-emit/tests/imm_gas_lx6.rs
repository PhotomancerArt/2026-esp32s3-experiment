//! LX6 vs LX7 assembler conformance for the immediate-legality table (P6).
//!
//! `src/imm.rs` was originally verified against the **S3 (LX7)** assembler
//! only; its "identical on LX6" note was an ISA-level claim. This test makes
//! the claim live: every boundary case is assembled with BOTH
//! `xtensa-esp32-elf-as` (classic ESP32, LX6) and `xtensa-esp32s3-elf-as`
//! (S3, LX7), `--no-transform`, asserting
//!
//! 1. the two assemblers return the same accept/reject verdict,
//! 2. that verdict matches the pinned expectation (the table's rule, plus the
//!    two documented gas quirks on `slli`), and
//! 3. where both accept, the emitted instruction bytes are identical.
//!
//! PC-relative entries (branches, `j`, `call0/4/8/12`, `l32r`) are probed by
//! laying out a target label at the exact displacement with `.space` padding,
//! so the reach limits themselves are exercised, including `l32r`'s
//! one-extended backward half (field 0x7FFF => -131076) and `call`
//! target alignment.
//!
//! Skips (loudly) when the espup Xtensa toolchain is not installed; override
//! the location with `XT_XTENSA_GAS_DIR` (a dir containing both `*-as`
//! binaries). First verified 2026-07-28 against crosstool-NG
//! esp-14.2.0_20240906 (binutils 2.43.1): 171 cases, zero divergences.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// One probe: a source file, the expected gas verdict, and the offset of the
/// probed instruction inside `.text` (for byte comparison).
struct Case {
    name: String,
    src: String,
    accept: bool,
    inst_off: usize,
}

/// A single-instruction case at offset 0.
fn simple(cases: &mut Vec<Case>, name: impl Into<String>, line: &str, accept: bool) {
    cases.push(Case {
        name: name.into(),
        src: format!("\t{line}\n"),
        accept,
        inst_off: 0,
    });
}

/// A multi-line layout case (PC-relative reach probes).
fn layout(cases: &mut Vec<Case>, name: impl Into<String>, src: impl Into<String>, accept: bool, inst_off: usize) {
    cases.push(Case {
        name: name.into(),
        src: src.into(),
        accept,
        inst_off,
    });
}

/// Forward branch/jump probe: instruction at 0 (length `ilen`), target label
/// at displacement `disp` from `PC + 4`.
fn fwd(cases: &mut Vec<Case>, name: &str, inst: &str, ilen: usize, disp: usize, accept: bool) {
    let pad = 4 + disp - ilen;
    layout(cases, name, format!("\t{inst} T\n\t.space {pad}\nT:\n"), accept, 0);
}

/// Backward branch/jump probe: target label at 0, instruction placed so its
/// displacement from `PC + 4` is `-disp`.
fn bwd(cases: &mut Vec<Case>, name: &str, inst: &str, disp: usize, accept: bool) {
    let pad = disp - 4;
    layout(cases, name, format!("T:\n\t.space {pad}\n\t{inst} T\n"), accept, pad);
}

/// The boundary corpus. Mirrors `tests/imm_legality.rs`'s pinned ranges; the
/// expectations are the gas verdicts (identical to the table everywhere
/// except the two `slli` quirks, documented inline).
fn build_cases() -> Vec<Case> {
    let mut c = Vec::new();
    // -- constant materialization / add --
    for (v, ok) in [(-128, true), (0, true), (127, true), (-129, false), (128, false)] {
        simple(&mut c, format!("addi {v}"), &format!("addi a2, a3, {v}"), ok);
    }
    for (v, ok) in [(-1, true), (1, true), (15, true), (-2, false), (0, false), (16, false)] {
        simple(&mut c, format!("addi.n {v}"), &format!("addi.n a2, a3, {v}"), ok);
    }
    for (v, ok) in [
        (-32768, true), (-256, true), (0, true), (256, true), (32512, true),
        (-33024, false), (-255, false), (255, false), (32768, false),
    ] {
        simple(&mut c, format!("addmi {v}"), &format!("addmi a2, a3, {v}"), ok);
    }
    for (v, ok) in [(-2048, true), (0, true), (2047, true), (-2049, false), (2048, false)] {
        simple(&mut c, format!("movi {v}"), &format!("movi a2, {v}"), ok);
    }
    for (v, ok) in [(-32, true), (0, true), (95, true), (-33, false), (96, false)] {
        simple(&mut c, format!("movi.n {v}"), &format!("movi.n a2, {v}"), ok);
    }
    // -- THE key Xtensa fact: no bitwise-immediate forms, on either core --
    simple(&mut c, "andi", "andi a2, a3, 1", false);
    simple(&mut c, "ori", "ori a2, a3, 1", false);
    simple(&mut c, "xori", "xori a2, a3, 1", false);
    // -- load/store offsets (unsigned, scaled) --
    for op in ["l8ui", "s8i"] {
        for (v, ok) in [(0, true), (255, true), (-1, false), (256, false)] {
            simple(&mut c, format!("{op} {v}"), &format!("{op} a2, a3, {v}"), ok);
        }
    }
    for op in ["l16ui", "l16si", "s16i"] {
        for (v, ok) in [(0, true), (2, true), (510, true), (1, false), (511, false), (512, false)] {
            simple(&mut c, format!("{op} {v}"), &format!("{op} a2, a3, {v}"), ok);
        }
    }
    for op in ["l32i", "s32i"] {
        for (v, ok) in [
            (0, true), (4, true), (1020, true),
            (1, false), (2, false), (1021, false), (1024, false),
        ] {
            simple(&mut c, format!("{op} {v}"), &format!("{op} a2, a3, {v}"), ok);
        }
    }
    for op in ["l32i.n", "s32i.n"] {
        for (v, ok) in [(0, true), (60, true), (2, false), (61, false), (64, false)] {
            simple(&mut c, format!("{op} {v}"), &format!("{op} a2, a3, {v}"), ok);
        }
    }
    // -- entry frame field (imm12 scaled by 8) --
    for (v, ok) in [(0, true), (8, true), (32760, true), (7, false), (32761, false), (32768, false)] {
        simple(&mut c, format!("entry {v}"), &format!("entry a1, {v}"), ok);
    }
    // -- shifts / extract / sext --
    // gas quirks (identical on both cores, deliberately NOT the table's rule,
    // which follows LLVM): sa=0 is rejected under --no-transform (the table
    // also treats it as illegal -> `mov`), and sa=32 is *accepted* by gas
    // (field 0) while the table treats it as illegal (-> `movi 0`).
    for (v, ok) in [(1, true), (31, true), (0, false), (32, true)] {
        simple(&mut c, format!("slli {v}"), &format!("slli a2, a3, {v}"), ok);
    }
    for (v, ok) in [(0, true), (15, true), (16, false)] {
        simple(&mut c, format!("srli {v}"), &format!("srli a2, a3, {v}"), ok);
    }
    for (v, ok) in [(0, true), (31, true), (32, false)] {
        simple(&mut c, format!("srai {v}"), &format!("srai a2, a3, {v}"), ok);
        simple(&mut c, format!("ssai {v}"), &format!("ssai {v}"), ok);
        simple(&mut c, format!("bbci {v}"), &format!("bbci a2, {v}, ."), ok);
    }
    for (s, w, ok) in [
        (0, 1, true), (0, 16, true), (16, 16, true), (31, 1, true), (24, 8, true),
        (17, 16, false), (25, 8, false), (32, 1, false), (0, 17, false), (0, 0, false),
    ] {
        simple(&mut c, format!("extui {s},{w}"), &format!("extui a2, a3, {s}, {w}"), ok);
    }
    for (v, ok) in [(7, true), (22, true), (6, false), (23, false)] {
        simple(&mut c, format!("sext {v}"), &format!("sext a2, a3, {v}"), ok);
    }
    // -- option presence: DIV32 / MUL32 / MUL32H exist on both cores --
    for op in ["quos", "quou", "rems", "remu", "mull", "muluh", "mulsh"] {
        simple(&mut c, format!("opt {op}"), &format!("{op} a2, a3, a4"), true);
    }
    // -- b4const / b4constu membership --
    for (v, ok) in [(-1, true), (1, true), (2, true), (256, true), (0, false), (9, false)] {
        simple(&mut c, format!("beqi {v}"), &format!("beqi a2, {v}, ."), ok);
    }
    for (v, ok) in [
        (2, true), (32768, true), (65536, true),
        (0, false), (1, false), (32767, false), (65535, false),
    ] {
        simple(&mut c, format!("bltui {v}"), &format!("bltui a2, {v}, ."), ok);
    }
    // -- branch reach: RRI8 (+-128 from PC+4), BRI12 (+-2048), narrow (0..63) --
    fwd(&mut c, "beq +127", "beq a2, a3,", 3, 127, true);
    fwd(&mut c, "beq +128", "beq a2, a3,", 3, 128, false);
    bwd(&mut c, "beq -128", "beq a2, a3,", 128, true);
    bwd(&mut c, "beq -129", "beq a2, a3,", 129, false);
    fwd(&mut c, "beqz +2047", "beqz a2,", 3, 2047, true);
    fwd(&mut c, "beqz +2048", "beqz a2,", 3, 2048, false);
    bwd(&mut c, "beqz -2048", "beqz a2,", 2048, true);
    bwd(&mut c, "beqz -2049", "beqz a2,", 2049, false);
    fwd(&mut c, "beqz.n +0", "beqz.n a2,", 2, 0, true);
    fwd(&mut c, "beqz.n +63", "beqz.n a2,", 2, 63, true);
    fwd(&mut c, "beqz.n +64", "beqz.n a2,", 2, 64, false);
    layout(&mut c, "beqz.n -4", "T:\n\tbeqz.n a2, T\n", false, 0); // forward-only
    // -- j: signed 18-bit byte displacement --
    fwd(&mut c, "j +131071", "j", 3, 131071, true);
    fwd(&mut c, "j +131072", "j", 3, 131072, false);
    bwd(&mut c, "j -131072", "j", 131072, true);
    bwd(&mut c, "j -131073", "j", 131073, false);
    // -- calls: signed 18-bit WORD displacement from (PC & !3) + 4 --
    for op in ["call0", "call4", "call8", "call12"] {
        layout(
            &mut c,
            format!("{op} +524284"),
            format!("\t{op} T\n\t.space 524285\nT:\n"),
            true,
            0,
        );
        layout(
            &mut c,
            format!("{op} +524288"),
            format!("\t{op} T\n\t.space 524289\nT:\n"),
            false,
            0,
        );
    }
    layout(&mut c, "call8 -524288", "T:\n\t.space 524284\n\tcall8 T\n", true, 524284);
    layout(&mut c, "call8 -524292", "T:\n\t.space 524288\n\tcall8 T\n", false, 524288);
    layout(&mut c, "call8 misaligned", "\tcall8 T\n\t.space 3\nT:\n", false, 0);
    // -- l32r: backward only, one-extended (full reach -262144..=-4) --
    layout(&mut c, "l32r -4", "T:\t.word 0x12345678\n\tl32r a2, T\n", true, 4);
    layout(
        &mut c,
        "l32r -131076 (one-extended half)",
        "T:\t.word 0x12345678\n\t.space 131072\n\tl32r a2, T\n",
        true,
        131076,
    );
    layout(
        &mut c,
        "l32r -262144",
        "T:\t.word 0x12345678\n\t.space 262140\n\tl32r a2, T\n",
        true,
        262144,
    );
    layout(
        &mut c,
        "l32r -262148",
        "T:\t.word 0x12345678\n\t.space 262144\n\tl32r a2, T\n",
        false,
        262148,
    );
    layout(
        &mut c,
        "l32r forward",
        "\tl32r a2, T\n\t.space 4\nT:\t.word 0x12345678\n",
        false,
        0,
    );
    c
}

/// Per-core toolchain (gas + objcopy).
struct Tools {
    gas: PathBuf,
    objcopy: PathBuf,
}

/// Locate a bin dir holding both cores' assemblers: `XT_XTENSA_GAS_DIR`, then
/// the espup install location, then `$PATH`.
fn find_tools() -> Option<(Tools, Tools)> {
    let mk = |dir: Option<&Path>, chip: &str| -> Tools {
        let name = |tool: &str| format!("xtensa-{chip}-elf-{tool}");
        match dir {
            Some(d) => Tools {
                gas: d.join(name("as")),
                objcopy: d.join(name("objcopy")),
            },
            None => Tools {
                gas: PathBuf::from(name("as")),
                objcopy: PathBuf::from(name("objcopy")),
            },
        }
    };
    let usable = |t: &Tools| {
        Command::new(&t.gas)
            .arg("--version")
            .output()
            .is_ok_and(|o| o.status.success())
    };
    let mut dirs: Vec<Option<PathBuf>> = Vec::new();
    if let Ok(d) = std::env::var("XT_XTENSA_GAS_DIR") {
        dirs.push(Some(PathBuf::from(d)));
    }
    if let Ok(home) = std::env::var("HOME") {
        let root = Path::new(&home).join(".rustup/toolchains/esp/xtensa-esp-elf");
        if let Ok(entries) = fs::read_dir(root) {
            for e in entries.flatten() {
                dirs.push(Some(e.path().join("xtensa-esp-elf/bin")));
            }
        }
    }
    dirs.push(None); // bare names on $PATH
    for dir in dirs {
        let lx6 = mk(dir.as_deref(), "esp32");
        let lx7 = mk(dir.as_deref(), "esp32s3");
        if usable(&lx6) && usable(&lx7) {
            return Some((lx6, lx7));
        }
    }
    None
}

/// Assemble `src`; on accept, return the 4 `.text` bytes at `inst_off`
/// (instructions are 2 or 3 bytes; trailing padding compares equal too).
fn assemble(tools: &Tools, src: &str, inst_off: usize, work: &Path, tag: &str) -> Option<Vec<u8>> {
    let s = work.join(format!("{tag}.s"));
    let o = work.join(format!("{tag}.o"));
    let b = work.join(format!("{tag}.bin"));
    fs::write(&s, src).expect("write asm source");
    let st = Command::new(&tools.gas)
        .arg("--no-transform")
        .arg("-o")
        .arg(&o)
        .arg(&s)
        .output()
        .expect("run gas");
    if !st.status.success() {
        return None;
    }
    let st = Command::new(&tools.objcopy)
        .args(["-O", "binary", "-j", ".text"])
        .arg(&o)
        .arg(&b)
        .output()
        .expect("run objcopy");
    assert!(st.status.success(), "objcopy failed for {tag}");
    let bytes = fs::read(&b).expect("read raw text");
    let end = (inst_off + 4).min(bytes.len());
    Some(bytes[inst_off..end].to_vec())
}

/// The conformance sweep: LX6 gas == LX7 gas == pinned verdict, byte-identical
/// encodings. This is the live version of `imm.rs`'s LX6 note.
#[test]
fn lx6_and_lx7_assemblers_agree_on_every_boundary() {
    let Some((lx6, lx7)) = find_tools() else {
        eprintln!(
            "SKIP imm_gas_lx6: xtensa-esp32-elf-as / xtensa-esp32s3-elf-as not found \
             (install the espup toolchain or set XT_XTENSA_GAS_DIR)"
        );
        return;
    };
    let work = std::env::temp_dir().join(format!("xt-imm-gas-lx6-{}", std::process::id()));
    fs::create_dir_all(&work).expect("create work dir");
    let mut failures = Vec::new();
    for case in build_cases() {
        let r6 = assemble(&lx6, &case.src, case.inst_off, &work, "lx6");
        let r7 = assemble(&lx7, &case.src, case.inst_off, &work, "lx7");
        match (&r6, &r7) {
            (Some(b6), Some(b7)) => {
                if !case.accept {
                    failures.push(format!("{}: expected reject, both accepted", case.name));
                } else if b6 != b7 {
                    failures.push(format!(
                        "{}: encodings differ lx6={b6:02x?} lx7={b7:02x?}",
                        case.name
                    ));
                }
            }
            (None, None) => {
                if case.accept {
                    failures.push(format!("{}: expected accept, both rejected", case.name));
                }
            }
            (a, b) => failures.push(format!(
                "{}: LX6/LX7 verdicts DIVERGE (lx6 accept={}, lx7 accept={})",
                case.name,
                a.is_some(),
                b.is_some()
            )),
        }
    }
    fs::remove_dir_all(&work).ok();
    assert!(
        failures.is_empty(),
        "assembler conformance failures:\n{}",
        failures.join("\n")
    );
}
