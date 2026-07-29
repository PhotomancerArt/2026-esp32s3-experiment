//! C2 — code-execution model discovery. THE deliverable of this spike.
//!
//! Read-back probes first (C2a RTC fast, C2b SRAM1 mapping shape, C2c SRAM0
//! data-writability), then execute probes (C2x: GV1 in each region that
//! passed its read-back, RTC → SRAM0 → SRAM1, plus a no-barrier rerun C2n).
//! Every risky step prints *before* it runs so a fault localizes.
//!
//! Faulting probes are data: esp-hal's `exception-handler` feature panics with
//! a full context dump (EXCCAUSE/EXCVADDR/PC), which flows through our C5
//! panic handler (print + RTC ledger + reboot).

use crate::codemem::{
    isync, memw, CodeSpot, RegionKind, Sram1Rule, RTC_DRAM_BASE, RTC_IRAM_BASE, SRAM1_DRAM_BASE,
    SRAM1_IRAM_BASE, SRAM1_IRAM_END,
};

/// Golden vector #1 (GV1), verbatim from FINDINGS.md — assembler-derived on
/// the S3 spike, byte-identical when re-assembled with xtensa-esp32-elf-as
/// (LX6): `entry a1, 32; movi a2, 42; retw` (wide forms).
pub const STUB42_BYTES: [u8; 9] = [0x36, 0x41, 0x00, 0x22, 0xa0, 0x2a, 0x90, 0x00, 0x00];

/// D-bus scratch base for SRAM1 shape probing — inside dram2_seg's free space
/// (0x3FFE_7E30..0x3FFF_FF80); nothing is linked there (only `.dram2_uninit`,
/// which this firmware doesn't use), and it is clear of the ROM data/stack
/// reservations lower in SRAM1.
const SRAM1_PROBE_DRAM: usize = 0x3FFF_0000;

/// SRAM0 scratch word, ~112KB into the 128KB iram_seg — far beyond this
/// firmware's few KB of `.rwtext`.
const SRAM0_PROBE_ADDR: usize = 0x4009_C000;

/// RTC-fast scratch offset — top quarter of the 8KB block, clear of the
/// `.rtc_fast.*` sections (the C5 ledger statics, 16 bytes at the bottom).
const RTC_PROBE_OFF: usize = 0x1800;

/// I-bus bases for GV1 execute probes (each region gets fresh addresses,
/// distinct from the read-back scratch, all word-aligned).
const RTC_EXEC_IRAM: usize = RTC_IRAM_BASE + 0x1900;
const SRAM0_EXEC_IRAM: usize = SRAM0_PROBE_ADDR + 0x100;
const SRAM1_EXEC_IRAM: usize = 0x400B_0800;
const SRAM1_NOBARRIER_IRAM: usize = 0x400B_0900;

pub struct Outcome {
    pub rtc_ok: bool,
    pub sram1_rule: Option<Sram1Rule>,
    pub sram0_writable: bool,
    /// Preferred region for C3/C4 (largest usable first).
    pub primary: Option<RegionKind>,
}

fn write_u32(addr: usize, v: u32) {
    // SAFETY: probe addresses are word-aligned scratch locations chosen to be
    // clear of every linker-placed section (see the consts above).
    unsafe { (addr as *mut u32).write_volatile(v) };
}

fn read_u32(addr: usize) -> u32 {
    // SAFETY: same address validity argument as `write_u32`.
    unsafe { (addr as *const u32).read_volatile() }
}

/// C2a — RTC fast: write via DRAM view, read via IRAM view, expect 1:1.
fn probe_rtc() -> bool {
    let offs = [RTC_PROBE_OFF, RTC_PROBE_OFF + 4, RTC_PROBE_OFF + 0x10];
    for off in offs {
        write_u32(RTC_DRAM_BASE + off, 0xA110_0000 | off as u32);
    }
    memw();
    esp_println::println!(
        "C2a: probing RTC-fast I-bus read at {:#x} (fault here = no I-bus data reads)",
        RTC_IRAM_BASE + RTC_PROBE_OFF
    );
    let mut ok = true;
    for off in offs {
        let want = 0xA110_0000 | off as u32;
        let got = read_u32(RTC_IRAM_BASE + off);
        esp_println::println!("C2a: MEASURE off={off:#x} want={want:#x} got={got:#x}");
        ok &= got == want;
    }
    esp_println::println!(
        "C2a: {} rtc_mapping={}",
        if ok { "PASS" } else { "FAIL" },
        if ok { "1to1" } else { "unknown" }
    );
    ok
}

/// C2b — SRAM1 mapping shape: distinct sentinels at several DRAM offsets,
/// then read the H1 (linear) and H2 (word-mirrored) I-bus candidates for each.
fn probe_sram1() -> Option<Sram1Rule> {
    let offs = [0usize, 4, 8, 0x40, 0x100];
    for off in offs {
        write_u32(SRAM1_PROBE_DRAM + off, 0xC0DE_0000 | off as u32);
    }
    memw();
    // Sanity: the D-bus view reads back what we wrote.
    for off in offs {
        let want = 0xC0DE_0000 | off as u32;
        let got = read_u32(SRAM1_PROBE_DRAM + off);
        if got != want {
            esp_println::println!("C2b: FAIL reason=dbus_readback off={off:#x} got={got:#x}");
            return None;
        }
    }
    esp_println::println!(
        "C2b: probing SRAM1 I-bus reads (H1 linear / H2 word-mirrored; fault = no I-bus data reads)"
    );
    let base_delta = SRAM1_PROBE_DRAM - SRAM1_DRAM_BASE;
    let mut h1_hits = 0;
    let mut h2_hits = 0;
    for off in offs {
        let want = 0xC0DE_0000 | off as u32;
        let h1 = SRAM1_IRAM_BASE + base_delta + off;
        let h2 = (SRAM1_IRAM_END - 4) - (base_delta + off);
        let v1 = read_u32(h1);
        let v2 = read_u32(h2);
        esp_println::println!(
            "C2b: MEASURE off={off:#x} want={want:#x} h1@{h1:#x}={v1:#x} h2@{h2:#x}={v2:#x}"
        );
        h1_hits += (v1 == want) as u32;
        h2_hits += (v2 == want) as u32;
    }
    let n = offs.len() as u32;
    if h1_hits == n && h2_hits != n {
        esp_println::println!("C2b: PASS sram1_rule=linear iram=0x400A0000+(dram-0x3FFE0000)");
        Some(Sram1Rule::Linear)
    } else if h2_hits == n && h1_hits != n {
        esp_println::println!(
            "C2b: PASS sram1_rule=word_mirrored iram=0x400BFFFC-(dram-0x3FFE0000)"
        );
        Some(Sram1Rule::WordMirrored)
    } else {
        esp_println::println!("C2b: FAIL reason=no_unique_rule h1_hits={h1_hits} h2_hits={h2_hits}");
        None
    }
}

/// C2c — SRAM0: is the I-bus-only segment data-writable with aligned words?
fn probe_sram0() -> bool {
    esp_println::println!(
        "C2c: probing SRAM0 word write at {SRAM0_PROBE_ADDR:#x} (fault = IRAM not data-writable)"
    );
    write_u32(SRAM0_PROBE_ADDR, 0xF00D_FACE);
    write_u32(SRAM0_PROBE_ADDR + 4, 0x1BAD_B002);
    memw();
    let a = read_u32(SRAM0_PROBE_ADDR);
    let b = read_u32(SRAM0_PROBE_ADDR + 4);
    let ok = a == 0xF00D_FACE && b == 0x1BAD_B002;
    esp_println::println!(
        "C2c: {} word_write=ok got0={a:#x} got1={b:#x}",
        if ok { "PASS" } else { "FAIL" }
    );
    ok
}

/// Write GV1 into `spot` and call it, expecting 42. `label` is the serial tag.
fn exec_gv1(label: &str, spot: CodeSpot, barriers: bool) -> bool {
    esp_println::println!(
        "{label}: probing exec region={} write0={:#x} exec={:#x} barriers={}",
        spot.kind.name(),
        spot.write_addr(0),
        spot.exec_addr(0),
        if barriers { "memw+isync" } else { "none" }
    );
    spot.write_code(&STUB42_BYTES);
    // Read-back sanity through the write view before jumping.
    let w0 = spot.read_word(0);
    if w0 != u32::from_le_bytes([0x36, 0x41, 0x00, 0x22]) {
        esp_println::println!("{label}: FAIL reason=code_readback w0={w0:#x}");
        return false;
    }
    if barriers {
        memw();
        isync();
    }
    // SAFETY: the spot holds a complete windowed function (GV1: entry/retw),
    // word-aligned, whose I-bus address is exec_addr(0); calling it via the
    // windowed extern "C" ABI is well-formed. If the region is not fetchable
    // the resulting exception is the datum this probe exists to collect.
    let f: extern "C" fn() -> u32 = unsafe { core::mem::transmute(spot.exec_addr(0)) };
    let v = f();
    let ok = v == 42;
    esp_println::println!(
        "{label}: {} region={} value={v} exec_addr={:#x}",
        if ok { "PASS" } else { "FAIL" },
        spot.kind.name(),
        spot.exec_addr(0)
    );
    ok
}

pub fn run() -> Outcome {
    let rtc_ok = probe_rtc();
    let sram1_rule = probe_sram1();
    let sram0_writable = probe_sram0();

    // Execute probes, safest claim first (RTC is marked RWX in the linker).
    let mut rtc_exec = false;
    let mut sram0_exec = false;
    let mut sram1_exec = false;
    if rtc_ok {
        rtc_exec = exec_gv1("C2x", CodeSpot::new(RegionKind::RtcFast, RTC_EXEC_IRAM), true);
    }
    if sram0_writable {
        sram0_exec = exec_gv1("C2x", CodeSpot::new(RegionKind::Sram0, SRAM0_EXEC_IRAM), true);
    }
    if let Some(rule) = sram1_rule {
        let spot = CodeSpot::new(RegionKind::Sram1(rule), SRAM1_EXEC_IRAM);
        sram1_exec = exec_gv1("C2x", spot, true);
        if sram1_exec {
            // C2n — same region, fresh address, no barriers (internal SRAM is
            // expected uncached on classic too; recording the answer).
            exec_gv1(
                "C2n",
                CodeSpot::new(RegionKind::Sram1(rule), SRAM1_NOBARRIER_IRAM),
                false,
            );
        }
    }

    let primary = if sram1_exec {
        sram1_rule.map(RegionKind::Sram1)
    } else if sram0_exec {
        Some(RegionKind::Sram0)
    } else if rtc_exec {
        Some(RegionKind::RtcFast)
    } else {
        None
    };
    match primary {
        Some(kind) => esp_println::println!("C2: PASS primary_region={}", kind.name()),
        None => esp_println::println!("C2: FAIL reason=no_executable_region"),
    }
    Outcome { rtc_ok, sram1_rule, sram0_writable, primary }
}

/// Sacrificial fault probe: byte store into SRAM0 (feature `probe-iram-byte`).
/// EXPECTED to raise a LoadStoreError-class exception (word-only bus).
#[cfg(feature = "probe-iram-byte")]
pub fn fault_probe_iram_byte() {
    write_u32(SRAM0_PROBE_ADDR, 0x5555_5555);
    esp_println::println!(
        "C2f: probing SRAM0 BYTE write at {:#x} (expect LoadStoreError)",
        SRAM0_PROBE_ADDR + 1
    );
    // SAFETY: not safe — deliberately issuing a byte-granularity store to the
    // instruction bus to observe the exception. Feature-gated sacrificial run.
    unsafe { ((SRAM0_PROBE_ADDR + 1) as *mut u8).write_volatile(0xAA) };
    let got = read_u32(SRAM0_PROBE_ADDR);
    esp_println::println!("C2f: SURPRISE byte write did not fault readback={got:#x}");
}

/// Sacrificial fault probe: execute GV1 from its D-bus SRAM1 address
/// (feature `probe-identity-exec`). EXPECTED to raise an instruction-fetch
/// exception — D-bus addresses should not be fetchable.
#[cfg(feature = "probe-identity-exec")]
pub fn fault_probe_identity_exec() {
    let dram = SRAM1_PROBE_DRAM + 0x400;
    let mut w = [0u8; 12];
    w[..9].copy_from_slice(&STUB42_BYTES);
    for (i, chunk) in w.chunks_exact(4).enumerate() {
        write_u32(dram + 4 * i, u32::from_le_bytes(chunk.try_into().unwrap()));
    }
    memw();
    isync();
    esp_println::println!("C2g: probing identity exec at {dram:#x} (expect InstrFetch fault)");
    // SAFETY: not safe — deliberately jumping to a D-bus address to observe
    // the exception. Feature-gated sacrificial run.
    let f: extern "C" fn() -> u32 = unsafe { core::mem::transmute(dram) };
    let v = f();
    esp_println::println!("C2g: SURPRISE identity exec worked value={v}");
}
