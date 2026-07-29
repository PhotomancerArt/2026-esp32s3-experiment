# xt-runner-esp32

Resident **classic ESP32 (LX6)** firmware that executes Xtensa machine-code
payloads sent over UART0 (through the board's USB-UART bridge) — **without
reflashing**. The classic sibling of
[`fw/xt-runner-esp32s3`](../xt-runner-esp32s3): together they are the hardware
oracles the emulator (`lp-xt-emu`) and emitter (`xt-mini-emit`) diff against.

Pairs with [`xt-runner-client`](../../xt-runner-client) (host) and
[`xt-runner-proto`](../../xt-runner-proto) (shared wire types; `DeviceInfo`
reports `chip: Esp32`). The board-agnostic logic — crash ledger, protocol
dispatch, payload flow, watchdog policy — lives in
[`xt-runner-core`](../../xt-runner-core); this crate supplies the classic
pieces:

- **Transport**: UART0 at **115200 8N1** (TX=GPIO1, RX=GPIO3) — classic ESP32
  has no native USB, and unlike USB-CDC the baud is load-bearing; the client
  must match (`src/board.rs`).
- **Code memory**: a **fixed 92 KiB SRAM1 region** written through the
  word-**mirrored** D-bus view (`src/codemem.rs`). The classic heap (SRAM2) is
  *not* executable — S3's "the heap is executable" does not carry (FINDINGS
  C2) — so `capacity()`/`TooLarge` genuinely bound payloads here.
- **Watchdog**: the RWDT with a `ResetCore` stage action (RTC RAM — and thus
  the crash ledger — survives a hang reset; C5 proved the classic ledger
  persists).
- **Ledger storage + panic handler**: the persistent RTC-fast statics (8KB at
  DRAM `0x3FF8_0000`, PRO_CPU only on classic) and the
  EXCCAUSE/EPC1/EXCVADDR-reading panic handler (`src/main.rs`).

## Transport notes (the classic-only gotchas, from FINDINGS C1/C5)

- **No esp-println anywhere in this crate.** The channel is pure binary
  (COBS-framed postcard), and esp-println's `uart` feature never programs the
  baud divisor after `esp_hal::init()` reclocks — output garbles at every
  baud. Driving `esp_hal::uart::Uart` directly (constructed after init)
  programs the divisor correctly.
- **`software_reset()` truncates an undrained UART TX FIFO.** The panic
  handler waits ~300ms before resetting so any in-flight frame drains
  (USB-CDC on the S3 was immune).
- **The bridge port does NOT drop across a device reset** (unlike the S3's
  USB-CDC re-enumeration) — but *opening* the port can itself reset the board
  (DTR/RTS auto-reset wiring). The client tolerates both behaviors.

## Code memory model

SRAM1's dual mapping is word-mirrored (FINDINGS C2b, hardware-measured):

```text
iram = 0x400B_FFFC − (dram − 0x3FFE_0000)      (word granularity)
```

The payload region — which **must stay in lockstep with `lp-xt-emu`'s
`BoardProfile::esp32()`**, or dual-run silently breaks:

| | D-bus (write) | I-bus (execute) |
|---|---|---|
| Region | `0x3FFE_8000 .. 0x3FFF_F000` (92 KiB) | `0x400A_1000 .. 0x400B_8000` |
| Payload byte 0 | `0x3FFF_EFFC` (the *last* word — the write walk runs downward) | `0x400A_1000` |

The writer is keyed on the I-bus layout (word `i` at `0x400A_1000 + 4*i`,
little-endian words verbatim, zero-padded) and computes the D-bus address per
word — the `CodeSpot` shape hardware-proven in `fw/spike-esp32` (C2/C3/C4).
The D-bus range sits inside esp-hal's `dram2_seg`, whose only linkable section
(`.dram2_uninit`) this firmware does not use.

## Protocol / crash model

Identical to the S3 runner (see its README): `Ping`/`Info`/`LoadExec`,
fault → panic handler records EXCCAUSE/EPC1/EXCVADDR → reset, hang → RWDT
`ResetCore`, next boot emits an unsolicited `CrashReport` correlated by `seq`.
ROM boot text lands on the same UART; the leading-COBS-delimiter send and the
client's skip-undecodable-frames behavior isolate it (verified at 115200).

## Build / flash / verify

```bash
cd fw/xt-runner-esp32
cargo build --release
espflash flash --chip esp32 --port /dev/cu.usbserial-1440 \
  target/xtensa-esp32-none-elf/release/xt-runner-esp32
XT_PORT_ESP32=/dev/cu.usbserial-1440 cargo test -p xt-runner-client -- --test-threads=1
```

Requires the Espressif `esp` Rust toolchain (see repo README). Xtensa asm
needs `#![feature(asm_experimental_arch)]`.

## Provenance

Original code. No derivation from third-party sources. The mirrored-SRAM1
writer and the classic code-memory model come from this repo's own C1–C5
hardware ladder (`fw/spike-esp32`, FINDINGS.md classic section); the shared
logic is `xt-runner-core` (see
`docs/adr/2026-07-28-runner-board-abstraction.md` and
`docs/adr/2026-07-28-license-provenance-discipline.md`).
