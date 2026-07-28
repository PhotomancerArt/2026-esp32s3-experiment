//! Traps: how a run stops abnormally, mirroring `xt_runner_proto::CrashReport`
//! so dual-run can compare emulator faults against hardware crash reports.

/// Classification of an abnormal stop.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TrapKind {
    /// A hardware-style exception (illegal instruction, bad fetch, bad
    /// load/store). Corresponds to `xt_runner_proto::CrashKind::Exception`.
    Exception,
    /// The instruction budget was exhausted — the payload looped forever.
    /// Corresponds to the device watchdog firing (`CrashKind::Timeout`).
    Timeout,
}

/// A trap raised during execution.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Trap {
    pub kind: TrapKind,
    /// EXCCAUSE-style cause code (0 for timeouts).
    pub cause: u32,
    /// Faulting PC (0 if not applicable / filled in by the run loop).
    pub pc: u32,
    /// Faulting data address for load/store errors (else 0).
    pub vaddr: u32,
}

/// EXCCAUSE for an illegal / unsupported instruction (`IllegalInstructionCause`).
pub const EXC_ILLEGAL_INSTRUCTION: u32 = 0;
/// EXCCAUSE for a `SYSCALL` with no host handler installed (`SyscallCause`).
pub const EXC_SYSCALL: u32 = 1;
