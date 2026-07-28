# xt-runner-client

Host-side client for [`xt-runner`](../fw/xt-runner): send Xtensa code payloads to
a resident ESP32-S3 and get back results or crash reports, without reflashing.

```rust
use xt_runner_client::{Runner, RunOutcome};

let mut r = Runner::open("/dev/cu.usbmodem1101")?;
r.ping()?;
// entry a1,32; movi a2,42; retw  -> returns 42
let code = vec![0x36,0x41,0x00, 0x22,0xa0,0x2a, 0x90,0x00,0x00];
match r.load_exec(1, code, 0, 0)? {
    RunOutcome::Ok(v)      => println!("result = {v}"),
    RunOutcome::Crash(rep) => println!("crashed: {rep:?}"),
}
```

`load_exec` handles crash recovery transparently: a faulting or hanging payload
resets the device (dropping the USB-CDC port), and the client reopens the port
and reads the `CrashReport` the firmware emits on its next boot, correlating it
by `seq`.

## Testing against hardware

Integration tests live in `tests/hardware.rs`, gated on `XT_DEVICE_PORT` (skipped
when unset, so plain `cargo test` stays green without a board). They must run
single-threaded — there is only one device:

```bash
cd fw/xt-runner && cargo run --release        # flash the runner first
XT_DEVICE_PORT=/dev/cu.usbmodem1101 \
  cargo test -p xt-runner-client --test hardware -- --test-threads=1 --nocapture
```

This same dual-run pattern (emulator always, hardware when `XT_DEVICE_PORT` is
set) is used by `lp-xt-emu` and `xt-mini-emit` for conformance.

## Provenance

Original code; see `docs/adr/2026-07-28-license-provenance-discipline.md`.
