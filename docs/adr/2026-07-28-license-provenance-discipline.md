# ADR: License provenance discipline for the Xtensa backend

- Status: accepted
- Date: 2026-07-28
- Deciders: Yona Appletree

## Context

This repository builds the lightplayer Xtensa backend — an instruction crate, an
emulator, an ELF loader, and a code emitter — targeting ESP32-S3. Building an ISA
backend invites copying from existing implementations, and the richest ones are
copyleft: QEMU (GPL-2.0), GNU binutils/GDB (GPL-3.0), and GCC (GPL-3.0).

lightplayer is licensed AGPL-3.0 **by choice, not necessity**. The project may later
wish to relicense (e.g. dual-license, or a commercial edition) while it still holds sole
copyright. Copying or transliterating GPL source into these crates would make that
choice irreversible: the code would carry an obligation that cannot be relicensed away,
permanently, regardless of the surrounding project's license.

This ADR is written **before any derivation work begins**, so it stands as dated
evidence that the discipline predates any contribution to these crates.

## Decision

**1. No GPL source, ever.** No code in this repository's Xtensa crates (`lp-xt-*`,
`xt-*`) may be copied, transliterated, or line-by-line adapted from any GPL-licensed
project. Named behavioral-reference-only projects:

- `espressif/qemu` (GPL-2.0) — may be run and observed as a behavioral oracle; its
  source may be read to understand semantics; **its code may not be reproduced**.
- `binutils-gdb` (GPL-3.0), including `xtensa-modules.c` and the linker's relocation
  handlers — same rule: read to understand, never reproduce.
- GCC (GPL-3.0) — same.

"Behavioral reference" means: observe inputs/outputs, understand the algorithm in the
abstract, then implement independently from primary specifications. It does not permit
translating a specific function into Rust.

**2. Apache-2.0-with-LLVM-exception derivation is permitted, with provenance.**
`espressif/llvm-project` (the Xtensa target's TableGen `.td` files) is Apache-2.0 WITH
LLVM-exception — compatible with relicensing. Encoding *data* (bit layouts, operand
fields, opcode values) may be derived from it, provided:

- each derived file carries a provenance header citing upstream repo, path, and commit
  SHA (template in `oss/XTENSA-REFS.md`);
- the upstream license text is vendored under `licenses/`
  (`LLVM-Apache-2.0-with-LLVM-exception.txt`);
- what is derived is factual encoding data, not creative code structure.

**3. Primary specifications are always safe.** The Xtensa ISA Reference Manual, the
Xtensa ISA Summary, and the ESP32-S3 TRM are the preferred sources for encodings,
semantics, and relocation formats — facts, not expression.

**4. Contribution intent.** Outside contributions to these crates should be accepted only
under a CLA or a DCO-with-explicit-license-grant, so the relicensing option is preserved
across contributors. (Policy to be formalized when the first outside contribution is
proposed; recorded here as intent.)

**5. Enforcement.** `AGENTS.md` restates rules 1–2 imperatively so automated agents
refuse GPL copying by default. Every crate README carries a Provenance section.

## Consequences

- The emulator and inst crate are implemented from the ISA manual + LLVM `.td` data +
  behavioral diffing against hardware and (optionally) QEMU — slower than copying QEMU,
  but relicensing-safe. This is a deliberate cost.
- When this work back-ports to the lp2025 monorepo, this ADR is mirrored/referenced
  there, since lp2025 is where outside AGPL contributions actually arrive and the same
  discipline must govern its Xtensa crates.
- If a future decision abandons the relicensing option, this ADR can be superseded and
  the GPL constraint relaxed — but that is a one-way door and must be explicit.

## Alternatives considered

- **Port QEMU's Xtensa core** (fastest emulator): rejected — permanent GPL encumbrance
  on the emulator, the single most reusable asset, killing relicensing.
- **Stay AGPL forever, copy freely**: rejected — forecloses a choice Yona wants to keep
  open, for a short-term implementation saving.
