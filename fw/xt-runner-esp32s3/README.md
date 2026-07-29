# xt-runner-esp32s3

Resident ESP32-S3 firmware that executes Xtensa machine-code payloads sent over
USB-Serial-JTAG — **without reflashing**. It is the *hardware oracle* for the
standalone Xtensa core: the emulator (`lp-xt-emu`) and emitter (`xt-mini-emit`)
diff their results against a real chip via this runner.

Pairs with [`xt-runner-client`](../../xt-runner-client) (host) and
[`xt-runner-proto`](../../xt-runner-proto) (shared wire types). The
board-agnostic logic — crash ledger, protocol dispatch, payload flow, watchdog
policy — lives in [`xt-runner-core`](../../xt-runner-core); this crate supplies
the S3-specific pieces:

- **Transport**: USB-Serial-JTAG (`src/board.rs`).
- **Code memory**: a heap buffer executed through SRAM1's uniform `+0x6F_0000`
  I-bus alias (`src/jitbuf.rs` — on the S3, "the heap is executable").
- **Watchdog**: the RWDT with a `ResetCore` stage action (RTC RAM — and thus
  the crash ledger — survives a hang reset).
- **Ledger storage + panic handler**: the persistent RTC-fast statics and the
  EXCCAUSE/EPC1/EXCVADDR-reading panic handler (`src/main.rs`).

## Protocol

Pure-binary channel: COBS-framed [postcard](https://docs.rs/postcard) messages
(zero byte = frame delimiter, so the host can resync after a device reset).
Nothing else may write to the USB serial FIFO — hence no `esp-println`.

- `Ping` → `Pong`
- `Info` → `DeviceInfo { proto_version, heap_free, max_payload, boot_count }`
- `LoadExec { seq, entry_offset, arg, code }` → copy `code` into an executable
  buffer, call `(buf + entry_offset)(arg)` as `extern "C" fn(u32) -> u32`, reply
  `Ok { seq, result }`.

## Crash model

A malformed payload must never brick the runner. Before jumping, the runner
records `seq` + a RUNNING marker in an RTC-fast-RAM ledger (survives resets) and
arms the RWDT watchdog. Then:

- **Fault** (bad fetch, load/store error, illegal instruction): esp-hal's
  exception handler raises a panic; the runner's panic handler reads the
  EXCCAUSE/EPC1/EXCVADDR special registers, records them, and `software_reset()`s.
- **Hang** (infinite loop): the watchdog resets the chip with the ledger still
  RUNNING.
- On the next boot the runner reads the ledger and emits an unsolicited
  `CrashReport { seq, kind, cause, pc, vaddr }` (kind = Exception / Timeout /
  Panic), then resumes accepting payloads. The client correlates it by `seq`
  across the port re-enumeration.

`pc` is the real faulting PC (EPC1) — exception frames are not window-mangled
(only a0 return addresses are), so no unmangling is needed.

## Build / flash

```bash
cd fw/xt-runner-esp32s3
cargo run --release          # builds, flashes, (no monitor — binary channel)
# or: espflash flash --chip esp32s3 --port <port> target/xtensa-esp32s3-none-elf/release/xt-runner-esp32s3
```

Requires the Espressif `esp` Rust toolchain (see repo README). Xtensa asm needs
`#![feature(asm_experimental_arch)]`.

## Provenance

Original code. No derivation from third-party sources. The JitBuf SRAM1-alias
technique and the RTC-ledger pattern come from this repo's own feasibility spike
(see `FINDINGS.md`); the shared logic was extracted to `xt-runner-core` in the
multi-board phase. See `docs/adr/2026-07-28-license-provenance-discipline.md`.
