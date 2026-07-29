# spike-esp32 — classic ESP32 (LX6) dynamic-code experiment ladder

Mirrors the S3 spike ([`fw/spike-esp32s3`](../spike-esp32s3)) for the **classic
ESP32** (LX6, rev v3.0, 4MB flash, dual-core, 240MHz), whose memory
architecture is fundamentally unlike the S3's. The ladder C1–C5 establishes,
on silicon, where dynamically written code can live and how to address it —
the finding that shapes the multi-board runner design. Verdicts and the full
code-execution model live in the repo [FINDINGS.md](../../FINDINGS.md)
("Classic ESP32" section).

## The ladder

- **C1** — toolchain + UART hello (esp-hal 1.1.1 / `esp32`, esp-println `uart`).
- **C2** — code-execution model discovery: read-back probes (C2a RTC fast 1:1,
  C2b SRAM1 mapping shape H1-linear vs H2-word-mirrored, C2c SRAM0
  data-writability), then GV1 execution in every region that passed (C2x),
  plus a no-barrier rerun (C2n). Feature-gated sacrificial fault probes:
  `probe-iram-byte` (byte store to SRAM0 — expect LoadStoreError),
  `probe-identity-exec` (execute at a D-bus address — expect InstrError).
- **C3** — windowed ABI: CALLX8 + L32R literal pool into a Rust builtin (GV2).
- **C4** — window overflow/underflow through emitted frames, depth 100 (GV3a/b).
- **C5** — abort-tier recovery (panic → RTC-fast ledger → reset → report) +
  heap/flash measurements.

All instruction bytes are the S3 spike's assembler-derived golden vectors
(FINDINGS.md); `xtensa-esp32-elf-as` re-assembly produces byte-identical
encodings for every wide-form instruction in them, and C2–C4 prove them on
LX6 silicon. Never hand-encode instructions.

## Flash + capture

```bash
cd fw/spike-esp32
cargo build --release
espflash flash --chip esp32 --port /dev/cu.usbserial-1440 \
  target/xtensa-esp32-none-elf/release/spike-esp32
# Real UART bridge: pass the baud explicitly (the port keeps stale speeds).
python3 ../../scripts/capture.py /dev/cu.usbserial-1440 12 115200
```

The board sits behind a USB-UART bridge (no native USB): the serial port does
**not** drop across device resets (one capture spans the C5 panic/reboot
cycle), but opening the port can itself reset the board via the auto-reset
wiring — use `espflash reset` for a deterministic full-boot capture.

## Classic-ESP32 gotchas baked into this crate

- **esp-println's `uart` feature never programs the baud divisor**; after
  `esp_hal::init()` reclocks the chip the ROM's divisor is stale and output
  turns to garbage. `main.rs` constructs a `UartTx` at 115200 first.
- **UART TX FIFO vs reset**: `software_reset()` right after a println
  truncates the message (unlike the S3's USB-CDC); the panic handler waits
  ~300ms so exception dumps survive.
- **Code writes are word-aligned 32-bit volatile stores** (`codemem.rs`);
  byte stores to I-bus addresses fault (LoadStoreError, verified).
