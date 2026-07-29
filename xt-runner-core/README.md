# xt-runner-core

Board-agnostic core of the **xt-runner** payload firmware — the resident
program that executes Xtensa machine-code payloads sent from a host, without
reflashing. Per-SOC firmware crates (`fw/xt-runner-esp32s3`, and the classic
`fw/xt-runner-esp32`) are thin shells over this crate: they supply only what
genuinely differs per chip (plus their own `Chip` id for `DeviceInfo`).

`no_std + alloc`, builds on stable Rust as a host workspace member, so the
ledger and dispatch logic are unit-tested off-device.

## What is shared (this crate)

- **Crash ledger logic** (`ledger`): build-id / boot-count / state / seq /
  cause / pc / vaddr cells; `boot()` (fresh-flash detection + crash report
  recovery), `arm()` / `disarm()` / `record_crash()`, and the
  `is_exception_cause` heuristic that distinguishes hardware faults from Rust
  panics.
- **Protocol plumbing** (`runner`): COBS frame accumulation with overrun
  resync, postcard dispatch of `Request` → `Response`, and the
  leading-delimiter send that isolates ROM boot-log noise into a discardable
  frame.
- **Payload flow + watchdog policy**: load → ledger-arm → watchdog-arm → sync
  → call → disarm, and the `PAYLOAD_WATCHDOG_MS` window.

## What each board supplies (the traits)

| Trait | S3 | Classic ESP32 |
|---|---|---|
| `Transport` | USB-Serial-JTAG | UART0 through the USB-UART bridge |
| `CodeMem` | heap buffer, executed at the `+0x6F_0000` I-bus alias | fixed SRAM1 region, word-writes through the **word-mirrored** D-bus view |
| `PayloadWatchdog` | RWDT stage 0, `ResetCore` | same policy, classic RWDT |

`CodeMem::load` takes the whole payload and returns the execute address,
rather than exposing a write pointer plus an alias offset, because the classic
chip's write walk is non-monotonic (mirrored SRAM1 writes run *backwards*
through the D-bus) and possibly word-only (SRAM0); no simple
"pointer + constant" contract survives both chips. `capacity()` +
`LoadError::TooLarge` let fixed-size regions bound the host.

`PayloadWatchdog` implementations must use a **core-only** reset
(`RwdtStageAction::ResetCore`, not the `enable()` default `ResetSystem`) — a
system reset also resets the RTC peripherals and wipes the ledger, so a hang
would be lost. The trait docs carry this contract; the per-board impls carry
the call.

## The ledger storage boundary

The ledger *logic* is here; the *placement* is the firmware's:

```rust
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LEDGER_CELLS: LedgerStorage = ledger_storage_init();
// ...
let ledger = Ledger::new(&LEDGER_CELLS, BUILD_ID);
```

Chosen over a chip-feature-gated attribute in core because (a) core then needs
no esp-hal dependency and stays a stable-Rust host crate with real unit tests,
and (b) RTC fast memory genuinely differs per chip (size, addresses, PRO_CPU
visibility on classic) — placement is exactly the thing a per-SOC crate should
own. `LedgerStorage` is a `[AtomicU32; 7]` rather than a named struct because
esp-hal's `persistent` attribute demands its unsafe `Persistable` trait, which
is implemented for atomics/arrays but cannot be implemented for a foreign
struct (orphan rule).

The panic handler also stays per-board: `#[panic_handler]` must live in the
final binary, and the EXCCAUSE/EPC1/EXCVADDR reads need Xtensa asm, which a
stable host-buildable crate cannot carry.

## Provenance

Original code, extracted from this repo's own `fw/xt-runner` (now
`fw/xt-runner-esp32s3`). No derivation from third-party sources. The RTC-ledger
pattern and code-memory models come from this repo's feasibility spikes (see
`FINDINGS.md`, E1–E5 and C1–C5). See
`docs/adr/2026-07-28-license-provenance-discipline.md`.
