# xt-runner-client

Host-side client for the `xt-runner` firmware family
([`fw/xt-runner-esp32s3`](../fw/xt-runner-esp32s3), [`fw/xt-runner-esp32`](../fw/xt-runner-esp32)):
send Xtensa code payloads to a resident board and get back results or crash
reports, without reflashing.

```rust
use xt_runner_client::{Runner, RunOutcome};

let mut r = Runner::open("/dev/cu.usbmodem1301")?;
r.ping()?;
// entry a1,32; movi a2,42; retw  -> returns 42
let code = vec![0x36,0x41,0x00, 0x22,0xa0,0x2a, 0x90,0x00,0x00];
match r.load_exec(1, code, 0, 0)? {
    RunOutcome::Ok(v)      => println!("result = {v}"),
    RunOutcome::Crash(rep) => println!("crashed: {rep:?}"),
}
```

`load_exec` handles crash recovery transparently on both transports: a faulting
or hanging payload resets the device; over USB-CDC (S3) the port drops and the
client reopens it, over a UART bridge (classic) the port stays open and the
client skips the boot noise — either way the firmware's next boot emits the
`CrashReport`, correlated by `seq`.

## Multi-board discovery (P5)

Boards are named by per-board env vars and **verified by chip id**, never by
port number (port names renumber across replug order — the S3 once moved
`usbmodem1101` → `usbmodem1301` when a third board appeared):

| Env var | Board | Transport |
|---|---|---|
| `XT_PORT_ESP32S3` (alias: `XT_DEVICE_PORT`) | ESP32-S3 (LX7) | USB-CDC (baud ignored) |
| `XT_PORT_ESP32` | classic ESP32 (LX6) | UART bridge @ 115200 |

`discover_boards()` returns one open, verified `Board` per configured var.
Failure semantics are deliberate:

- **Unset var = that board is skipped** (emulator-only tests stay green).
- **Configured-but-unreachable = hard error** — never a silent skip (silent
  skips hide regressions). A wedged board may need `espflash reset --port …`
  (an aborted flash/board-info can leave it in the ROM bootloader).
- **Reported chip ≠ expected chip = hard error** (`ChipMismatch`) — never a
  silent swap onto the wrong board.
- `XT_PORT_ESP32S3` and `XT_DEVICE_PORT` disagreeing is an error, and a
  firmware protocol-version mismatch says "reflash the runner".

## Testing against hardware

Integration tests live in `tests/hardware.rs`, gated on the port env vars
(skipped when all are unset, so plain `cargo test` stays green). They must run
single-threaded — each board is one shared resource:

```bash
cd fw/xt-runner-esp32s3 && cargo run --release     # flash the runner first
XT_PORT_ESP32S3=/dev/cu.usbmodem1301 \
  cargo test -p xt-runner-client --test hardware -- --test-threads=1 --nocapture
```

The N-run corpus (`xt-testkit`, used by `lp-xt-emu` and `xt-mini-emit` tests)
builds on this discovery: emulator on every board profile + every attached
board, one code path.

## Provenance

Original code; see `docs/adr/2026-07-28-license-provenance-discipline.md`.
