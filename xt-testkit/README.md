# xt-testkit

The shared **N-run** test harness (P5): every corpus case runs on the emulator
under **every** known `BoardProfile` (ESP32-S3 and classic ESP32 memory maps),
plus **every attached board** — each paired with the emulator on *its own*
profile — through one code path. Consumed as a dev-dependency by
`lp-xt-emu/tests/*` and `xt-mini-emit/tests/*` (a crate rather than a shared
module because the harness serves test suites in two crates).

## API sketch

```rust
let mut h = xt_testkit::Harness::from_env(1000);   // discovers boards, seq base
h.nrun("case", &code, entry, arg, expect);          // every world must Ok(expect)
h.nrun_expect("ill", &code, 0, 0, Expect::Crash(TrapKind::Exception));
let v = h.measure("probe", &code, entry, arg);      // all worlds agree; returns v
let per_world = h.run_all("raw", &code, entry, arg); // predicate-only comparisons
h.for_each_board(|b| { /* board-level tests */ });
```

Discovery semantics (see `xt-runner-client`): unset env var = board skipped;
configured-but-unreachable, wrong chip id, or wrong proto version = **panic**.
A case exceeding a world's payload capacity is skipped with a loud
`SKIP-CAPACITY` note naming the world and case — never truncated.

Board-specific expectations (capacity contracts, transport-quirk recovery)
live in `tests/board_esp32s3.rs` / `tests/board_esp32.rs`, gated on their own
board's env vars.

## Provenance

Original code; see `docs/adr/2026-07-28-license-provenance-discipline.md`.
