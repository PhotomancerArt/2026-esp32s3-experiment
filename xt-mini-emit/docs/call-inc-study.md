# CALL-increment policy study (P1)

Measured 2026-07-28 on the emulator (`lp-xt-emu`) **and** real ESP32-S3 silicon
(rev v0.2, `xt-runner` firmware via `xt-runner-client`). Rig:
[`tests/call_inc_study.rs`](../tests/call_inc_study.rs); emitter parameterization:
[`CallInc`](../src/emit.rs) (`emit_program_with(prog, inc)`; the default
`emit_program` stays byte-identical CALL8).

**Question.** `CALLn` rotates the register window by `n` at the callee's `ENTRY`
(callee `a0` = caller `a[n]`), and the callee's arguments are always its `a2..=a7`
(= caller `a[n+2]..a[n+7]`). The increment therefore simultaneously sets how many
caller registers survive a call, how many arguments fit in registers, and how fast
deep call chains exhaust the 64-register physical file. This choice literally writes
`caller_saved_pool_hw()` and `allocatable_pool_order()` for the lp2025 backport, so
it must be measured, not argued.

## Result summary

| | CALL4 | **CALL8** | CALL12 |
|---|---|---|---|
| Preserved across a call (measured, silicon) | `a2,a3` → **2** usable | `a2..a7` → **6** usable | `a2..a11` → **10** usable (8 in-window +2 beyond emitter scratch a8/a9) |
| Register-arg capacity (measured) | **6** (staged at caller `a6..a11`; 7 refused at emit) | **6** (caller `a10..a15`; 7 refused) | **2** (caller `a14,a15` only; **3 args refuse to emit**) |
| Arg staging hazard | staging area overlaps program regs `a6,a7` → emitter must bounce via `a12/a13` | none (disjoint) | none (disjoint) |
| First window-spill depth (recursion, runner-entered chain) | **11** | **6** | **4** |
| Steady-state spill traffic per extra frame | +1 trap / +4 regs | +1 trap / +8 regs | +1 trap / +12 regs |
| Frame save-area reservation | 32 B (16 hw-minimal; 32 = uniform floor, see below) | 32 B | **48 B** |

**Recommendation: CALL8.** The arithmetic prior is confirmed on silicon —
see "Recommendation" below for the reasoning against each alternative.

## 1. Register-argument capacity

Programs passing N arguments (order-sensitive shift-accumulate fold in the callee,
so permutation/staging bugs change the answer) were emitted under each increment and
dual-run for N = 1..capacity at 4 argument values including wrap-around cases.

- **CALL4: 6 args**, staged at caller `a6..a11`; all six verified
  emu == silicon == host-computed fold. **Finding:** the staging area overlaps the
  program registers (`a6`,`a7`), so the emitter must resolve the parallel-move
  hazard (this rig bounces those two through `a12/a13`, which sit above the CALL4
  staging area). A real backend needs that special case forever.
- **CALL8: 6 args**, staged at caller `a10..a15` — disjoint from program registers,
  no hazard. All six verified.
- **CALL12: 2 args.** The callee's `a2..a7` map to caller `a14..a19`, and the
  caller's window ends at `a15` — only `a14`,`a15` exist to stage into. A 3-argument
  call **cannot be emitted at all** (the rig pins the emit-time refusal). LPIR calls
  routinely carry 3+ scalar args, so CALL12 would force a stack-arg path onto
  *common* calls, not rare ones.

Capacity is `min(6, 16 − (4·inc + 2))`: 6 / 6 / 2. The "6" ceiling is the callee
side (args arrive in `a2..=a7`); the "2" is the caller's window edge.

## 2. Preserved temporaries (measured, not asserted)

For every register `a2..a15`, a probe plants a known constant, calls a callee that
actively overwrites **its entire window** (`a2..a15` — `movi` junk into all 14
writable registers), and checks survival after return. `a2..a7` probes go through
the emitter; `a8..a15` probes are built directly from `lp_xt_inst::encode`
(they're outside the MiniVInst program range). Every probe ran on emulator and
silicon; survival verdicts agreed on all 42 probes.

Measured survivors (of `a2..a15`):

| inc | survived | usable as allocator temporaries |
|---|---|---|
| CALL4 | `{a2,a3}` | **2** |
| CALL8 | `{a2,a3,a4,a5,a6,a7}` | **6** |
| CALL12 | `{a2..a11}` | **10** minus `a8`/`a9` emitter scratch → 8 program-usable |

The empirical rule, exactly as the ISA documents: caller `a{j}` survives iff
`j < 4·inc` (it sits below the callee's rotated window); `a[4·inc]` is destroyed by
the call itself (return address), `a[4·inc + 1]` by the callee's `ENTRY` (its SP).
A bonus confirmation fell out of the rig: the post-call `a8` under CALL8 holds the
**mangled return address** — top bits `0b10` = CALLINC 2 — whose low 30 bits are the
call-site PC, so the raw value is load-address-dependent (emulator and device load
code at different addresses). The survival *verdicts* still agreed everywhere; the
test compares the predicate, not the raw value.

## 3. Window-overflow onset and growth

Self-recursion `f(d) = d` (the M5 `prog_recursion` shape) emitted under each
increment, run at every depth 1..=40. Spill/reload events counted with `lp-xt-emu`'s
`TextTracer`; the emulator's window machinery is silicon-validated (FINDINGS.md:
depth-100 call8 and depth-60 call12 recursion produce identical results
emu-vs-device, and two window bugs were previously found by exactly this
dual-running). Hardware ran the same 120 program×depth points and returned the
correct result at every one — including well past each onset, so trap-driven
spill/reload round-trips are correct on silicon under all three increments.

Note the chain is runner-entered: frame 1 (the runner's context) and frame 2 (the
entry function, which the runner always reaches via CALL8) occupy 2 window units
each regardless of policy; recursion frames occupy `inc` units each.

| depth | CALL4 spills | CALL8 spills | CALL12 spills |
|---|---|---|---|
| 1–3 | 0 | 0 | 0 |
| 4 | 0 | 0 | **1** |
| 5 | 0 | 0 | 2 |
| 6 | 0 | **1** | 3 |
| 7 | 0 | 2 | 4 |
| 8 | 0 | 3 | 5 |
| 9 | 0 | 4 | 6 |
| 10 | 0 | 5 | 7 |
| 11 | **1** | 6 | 8 |
| 12 | 1 | 7 | 9 |
| 13 | 2 | 8 | 10 |
| 20 | 9 | 15 | 17 |
| 30 | 19 | 25 | 27 |
| 40 | 29 | 35 | 37 |

Reloads equal spills at every point (asserted). Growth is +1 trap per extra frame
in steady state for every increment; what differs is **onset** (11 / 6 / 4) and
**bytes per trap** — each spilled frame moves `4·inc` registers, so cumulative
traffic at depth 40 is 124 regs (CALL4) vs 280 (CALL8) vs 436 (CALL12): CALL12
pays ~3.5× CALL4's spill traffic and ~1.6× CALL8's, *and* starts paying ~7 frames
earlier than CALL4.

(CALL4's onset at 11 rather than ~14: the two call8-entered frames at the bottom of
the chain consume 4 of the 16 window units; and an `ENTRY`'s overflow check needs the
new frame's out-region free, not just its own units — both visible in the trace.)

## Frame save-area conclusion (worked from the ABI, encoded in `CallInc::save_area_bytes`)

A frame entered with increment `u` units (`u = n/4`) needs, at the top of its frame:

- the 16-byte **base save area** (an ancestor's `a0..a3` land there on overflow), plus
- `16·(u−1)` bytes for the **extra save area** — `_WindowOverflow8` spills the
  victim's `a4..a7` (16 B) and `_WindowOverflow12` its `a4..a11` (32 B) into the
  victim's own frame top.

So the hardware-minimal reservation is `16·u` bytes: **16 / 32 / 48** for
CALL4/8/12-entered frames. Two practical adjustments in this rig:

1. The entry function is always reached via the runner's CALL8, whatever the
   internal policy — so its reservation is floored at 32 B.
2. The reservation is applied uniformly (`16·max(2, u)`) rather than per-function,
   because `lp-xt-emu` models save areas as one contiguous `16·u` block below the
   *callee's* SP (observably equivalent for conforming frames, per its module docs) —
   uniform reservation keeps both the hardware handlers and the emulator model safe
   in mixed-increment chains (runner-CALL8 atop policy-CALLn). CALL4 therefore
   reserves 32 rather than its hardware-minimal 16 here; frame size does not affect
   any measured quantity (overflow onset is a register-file property).

Backport consequence (unchanged from BACKPORT.md, now with the constant pinned):
`abi/frame.rs::compute()` needs one ISA hook — reserved bytes at the top of the
frame — and under the recommended CALL8 policy that value is **32** (rv32 = 0).

## Recommendation: CALL8

- **Against CALL4:** only 2 registers survive a call. The allocator would have to
  treat 4 of its 6 program registers as caller-saved, spilling around *every* call —
  a per-call cost in the common path, bought back only as later trap onset (11 vs 6)
  in call chains deeper than ~6, which LPIR shader code (shallow call graphs,
  builtin-heavy) does not exhibit. It also carries the permanent `a6/a7` staging-
  hazard special case in the emitter.
- **Against CALL12:** 2 register args is disqualifying — 3-arg calls (common in
  LPIR) cannot be emitted without a stack-arg path on the hot path. It also traps
  earliest (depth 4), moves the most bytes per trap (48 B/frame), and grows every
  frame by 16 B of save-area reservation. The extra preserved registers (`a10,a11`
  beyond scratch) buy +2 temporaries over CALL8 — not worth the arg regression alone.
- **CALL8** is the only increment with no disqualifier: 6 preserved temporaries
  (near rv32's pool economics), 6 register args (equal best), conflict-free arg
  staging, mid-pack overflow onset with moderate per-trap cost, and the 32-byte
  frame reservation already hardware-proven by the spike and the M5 corpus.

For the P2 ABI contract this pins: `ARG_REGS` = callee `a2..=a7` (6),
`CALLER_SAVED`/staging = `a8..a15`, preserved program pool = `a2..=a7`,
frame top reservation = 32 bytes, call opcodes = `CALL8`/`CALLX8`.

## LX6 note (Q4)

Everything this study exercises — `CALL4/8/12`, `CALLX8`, `ENTRY`, `RETW`, the
`_WindowOverflow{4,8,12}`/`_WindowUnderflow*` vectors, MOVI/ADDI/L32R and the wide
ALU forms — is core Xtensa windowed-ABI machinery, present identically on LX6
(classic ESP32). Nothing in the measurement rig or the recommendation depends on an
LX7-only feature; the register file is 64 ARs on both, so the overflow-onset numbers
carry over. (The S3-specific parts of the repo — memory map, I/D-bus aliasing — are
runner concerns, not ABI concerns.)

## Reproduce

```bash
cargo test -p xt-mini-emit --test call_inc_study -- --nocapture          # emulator
XT_DEVICE_PORT=/dev/cu.usbmodem1101 \
  cargo test -p xt-mini-emit --test call_inc_study -- --test-threads=1 --nocapture
```

`MEASURE …` lines carry every number in this document.
