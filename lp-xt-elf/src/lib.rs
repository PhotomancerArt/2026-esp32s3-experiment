//! `lp-xt-elf` — loads linked Xtensa ELF32 executables into [`lp_xt_emu`]
//! memory and hosts the guest syscall ABI (print / exit / panic) that the
//! fixture corpus's `lp-xt-emu-guest` runtime targets.
//!
//! Scope (M4): **linked executables only** — `PT_LOAD` segments are copied to
//! their `p_vaddr` and the entry point is invoked; there is no relocation
//! processing (an ELF carrying REL/RELA sections is rejected with a clear
//! error — that is M6 territory).
//!
//! ELF parsing is delegated to the permissively-licensed `object` crate; this
//! crate contains no hand-rolled ELF code and no GPL-derived code (see the
//! repo license ADR and README Provenance section).

pub mod abi;
mod host;
mod loader;

pub use host::{run_elf, run_elf_traced, GuestHost, GuestRun};
pub use loader::{ElfError, Segment, XtensaElf};
