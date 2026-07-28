# lp-xt-elf

Loads **linked** Xtensa ELF32 executables into `lp-xt-emu` memory and hosts the
guest syscall ABI (print / exit / panic) used by the fixture corpus.

## What it does

- `XtensaElf::parse(bytes)` — validates ELF32, little-endian,
  `e_machine == EM_XTENSA` (94), object kind *Executable*, and **rejects any
  file with REL/RELA relocation sections** (linked executables are
  pre-resolved; relocation processing is deliberately out of scope until M6).
- `XtensaElf::load_into(&mut Emulator)` — copies each `PT_LOAD` segment to its
  `p_vaddr` (zero-filling the `p_memsz` tail for `.bss`), returning a clear
  error if a segment falls outside the emulator's modeled memory.
- `XtensaElf::entry()` / `XtensaElf::symbol(name)` — entry point + symbol
  lookup for test harnesses.
- `run_elf(bytes, arg)` — one-call harness: parse, load into a fresh
  `Emulator`, run from the ELF entry via the synthesized windowed CALL8
  (`Emulator::run_loaded`), with `GuestHost` handling syscalls. Returns a
  `GuestRun` (outcome, collected output, exit code, panic message).

## Guest syscall ABI

Defined in [`src/abi.rs`](src/abi.rs): the guest executes the `SYSCALL`
instruction with the syscall number in `a2` and arguments in `a3..a5`; the
host writes the result into `a2` and resumes (or terminates the run for
`SYS_EXIT` / `SYS_PANIC`). The guest-side mirror is
`fixtures/lp-xt-emu-guest`; the two constants files must stay in sync.

Address expectations for fixtures (see `fixtures/link.ld`): `.text` at
`0x40378000` (the I-bus alias of SRAM1 `0x3FC88000`), data at D-bus
`0x3FC98000` — both views of the emulator's modeled code region.

## Tests

- `tests/fixtures.rs` — runs every toolchain-compiled fixture ELF from
  `fixtures/elf/` (build with `fixtures/build.sh`; tests skip with a note when
  the ELFs are absent so the stable host workspace never needs the esp
  toolchain). Expected outputs are host-side oracles mirroring each guest
  program.
- `tests/loader_hosted.rs` — synthetic-ELF loader validation (segments, bss,
  rejection paths) with guest code assembled by `lp-xt-inst`'s encoder.

## Provenance

Original code. ELF parsing is delegated to the permissively-licensed `object`
crate (Apache-2.0/MIT), the same dependency `lp-xt-inst`'s objdiff uses; no
ELF handling was hand-rolled beyond reading `object`'s public API, and no GPL
source (binutils/GDB, QEMU) was consulted for code. See
`docs/adr/2026-07-28-license-provenance-discipline.md`.
