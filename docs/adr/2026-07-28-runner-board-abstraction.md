# ADR: xt-runner board abstraction — shared core, per-SOC crates

- Status: accepted
- Date: 2026-07-28
- Deciders: Yona Appletree
- Relates to: P2/P3 of the multi-board plan; `xt-runner-core` (the shared
  logic), `fw/xt-runner-esp32s3` (the S3 firmware), `FINDINGS.md` C1–C5 (the
  classic ESP32 memory model that shaped the traits)

## Context

The payload runner (formerly `fw/xt-runner`) mixed board-agnostic logic — the
RTC crash ledger, COBS/postcard protocol dispatch, the payload flow, the
watchdog policy — with two ESP32-S3-specific facts: USB-Serial-JTAG is the
transport, and "the heap is executable" via SRAM1's uniform `+0x6F_0000` I-bus
alias. The classic ESP32 (the real WLED target) invalidates both: it has a
USB-UART bridge instead of native USB, and P1 measured a radically different
code-memory model — the heap (SRAM2) is *not* executable; code goes to SRAM1
whose dual mapping is word-**mirrored** (`iram = 0x400BFFFC − (dram −
0x3FFE0000)`, the D-bus write walk runs backwards), to SRAM0 which accepts
32-bit-aligned word writes only, or to an 8KB RTC region.

## Decision

**One `xt-runner-core` crate holds the shared logic; each SOC family gets its
own thin firmware crate** (`fw/xt-runner-esp32s3`, `fw/xt-runner-esp32` in P3)
supplying board behavior through three traits:

- `Transport` — `read_byte() -> Option<u8>` / `write` / `flush`. Transports
  own their line errors; framing resyncs on COBS delimiters regardless.
- `CodeMem` — `load(&mut self, code: &[u8]) -> Result<usize, LoadError>` +
  `sync` + `release` + `capacity`. `load` takes the **whole payload** and
  returns only the execute address. It is deliberately *not* "a write pointer
  plus a constant alias offset": the classic chip's write walk is
  non-monotonic (mirrored SRAM1) and possibly word-only (SRAM0), and the
  returned I-bus address is unrelated to any write address by simple offset.
  Fixed-size regions report `LoadError::TooLarge`; `capacity()` bounds
  `DeviceInfo::max_payload`.
- `PayloadWatchdog` — `arm`/`disarm`, with the policy (the 3s window, the
  arm/disarm points in the payload flow, and the ResetCore-not-ResetSystem
  requirement) fixed in core and only the mechanism per-board.

The **ledger storage boundary**: core owns the ledger logic over a
`LedgerStorage` (`[AtomicU32; 7]`) that each firmware declares with its own
chip's persistent-RTC attribute. Chosen over a feature-gated attribute in core
because (a) core then has no esp-hal dependency, builds on stable, and joins
the host workspace with real unit tests; (b) RTC fast memory genuinely differs
per chip (size, addresses, PRO_CPU-only visibility on classic) — placement is
exactly what a per-SOC crate should own. The array (not a struct) is forced by
esp-hal's `Persistable` bound plus the orphan rule. The panic handler also
stays per-board: `#[panic_handler]` must live in the final binary and the
EXCCAUSE/EPC1/EXCVADDR reads need Xtensa asm a stable host crate cannot carry.

**Why per-SOC crates rather than feature flags**: the fw crates differ in
target triple, linker script, and HAL feature set (`esp32s3` vs `esp32`), so a
single crate would need mutually exclusive features *and* per-feature
`.cargo/config.toml` targets — which cargo cannot express; separate crates per
SOC family with their own toolchain pinning is the shape `fw/` already uses
(mirrors the earlier "one fw crate per SOC family" decision for spikes).

## Consequences

- P3's classic firmware supplies only: a UART transport, a fixed-region
  word-writing `CodeMem` (the `CodeSpot` shape from `fw/spike-esp32`), the
  classic RWDT arm, and its own ledger statics + panic handler.
- A third chip is configuration, not surgery: implement three traits and
  declare two statics.
- The wire protocol is untouched in P2 (the chip id lands in P3), so the
  existing `xt-runner-client` keeps working against the renamed S3 firmware.
- Load-bearing subtleties are pinned where they act: the ResetCore contract in
  the `PayloadWatchdog` docs (and the call site), the leading-COBS-delimiter
  trick in core's `send`, the RUNNING→CRASHED-only transition in the ledger.
