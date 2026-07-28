//! The emulator: memory + CPU + the run loop and the windowed-ABI run harness.

use crate::cpu::Cpu;
use crate::error::{Trap, TrapKind};
use crate::memory::Memory;
use crate::trace::{TraceEvent, Tracer};

/// Where payload code is placed in the emulator's SRAM1 (D-bus address). The
/// runner picks a heap address; we pick a fixed one inside the dual-mapped
/// window. Code executes at the I-bus alias of this address.
pub const CODE_DBUS_BASE: u32 = 0x3FC8_8000;
/// Size of the code region.
pub const CODE_REGION_LEN: usize = 0x0002_0000; // 128 KiB
/// Stack region (D-bus). Separate SRAM1-mapped region; stack grows down from
/// the top. Save areas produced by window spills live here.
pub const STACK_DBUS_BASE: u32 = 0x3FCC_0000;
pub const STACK_REGION_LEN: usize = 0x0002_0000; // 128 KiB
/// Initial stack pointer (top of the stack region, 16-aligned).
pub const INITIAL_SP: u32 = STACK_DBUS_BASE + STACK_REGION_LEN as u32 - 16;

/// Sentinel return address: when the top-level windowed function returns here,
/// the run stops. Chosen unmapped and in the code region's high bits so the
/// RETW address-unmangle reproduces it exactly (see `finish_call`).
pub const SENTINEL_PC: u32 = 0x4000_0000;

/// Default instruction budget before a run is declared a [`TrapKind::Timeout`]
/// (models the device watchdog catching an infinite loop). Far above any
/// payload the corpus runs; the hang case is the only one that reaches it.
pub const DEFAULT_STEP_BUDGET: u64 = 2_000_000;

/// Control-flow outcome of executing one instruction.
pub(crate) enum Flow {
    /// Advance to `pc + len`.
    Next,
    /// Jump to an absolute address (branch taken, call, return, jump).
    Jump(u32),
    /// A `SYSCALL` was executed: hand control to the run loop's
    /// [`SyscallHandler`] before advancing.
    Syscall,
}

/// What [`Emulator::step`] observed (beyond a trap).
enum Step {
    /// A normal instruction retired; `pc` already advanced.
    Normal,
    /// A `SYSCALL` retired; `pc` still points at it. `next_pc` is the
    /// instruction after it (where a resumed guest continues).
    Syscall { next_pc: u32 },
}

/// How a [`SyscallHandler`] tells the run loop to proceed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SyscallOutcome {
    /// Write this value to the guest's `a2` and continue after the `SYSCALL`.
    Resume(u32),
    /// Stop the run; the value becomes [`RunOutcome::Ok`]. (Guest `exit` — the
    /// handler records any abnormal detail, e.g. a panic message, itself.)
    Exit(u32),
}

/// Host hook invoked when the guest executes a `SYSCALL` instruction.
///
/// The guest ABI (which registers carry the syscall number/arguments) is the
/// handler's business, not the emulator's — the handler gets the full CPU and
/// memory. `lp-xt-elf` defines the ABI used by the fixture corpus.
pub trait SyscallHandler {
    fn syscall(&mut self, cpu: &mut Cpu, mem: &mut Memory) -> SyscallOutcome;
}

/// A completed run.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunOutcome {
    /// The top-level function returned; the value is its result register.
    Ok(u32),
    /// Execution trapped (exception or timeout).
    Trap(Trap),
}

/// The emulator.
pub struct Emulator {
    pub cpu: Cpu,
    pub mem: Memory,
    /// Instruction budget for [`RunOutcome::Trap`] timeout detection.
    pub step_budget: u64,
}

impl Emulator {
    /// Build an emulator with the standard S3 SRAM1 code + stack layout.
    pub fn new() -> Emulator {
        let mut mem = Memory::new();
        mem.add_sram1(CODE_DBUS_BASE, CODE_REGION_LEN);
        mem.add_sram1(STACK_DBUS_BASE, STACK_REGION_LEN);
        Emulator {
            cpu: Cpu::new(),
            mem,
            step_budget: DEFAULT_STEP_BUDGET,
        }
    }

    /// Run `code` as `fn(arg) -> u32`, entered at `entry_offset` within the
    /// blob, exactly as `xt-runner` does: the code is written to SRAM1 and
    /// executed at its I-bus alias, and the entry is invoked via a synthesized
    /// windowed CALL8 (arg staged in `a10`, arriving in the callee's `a2` after
    /// its ENTRY). Uses a no-op tracer.
    pub fn run(&mut self, code: &[u8], entry_offset: u32, arg: u32) -> RunOutcome {
        let mut t = crate::trace::NoopTracer;
        self.run_traced(code, entry_offset, arg, &mut t)
    }

    /// As [`run`](Self::run), emitting [`TraceEvent`]s to `tracer`.
    pub fn run_traced(
        &mut self,
        code: &[u8],
        entry_offset: u32,
        arg: u32,
        tracer: &mut dyn Tracer,
    ) -> RunOutcome {
        // Load code into SRAM1 at the fixed D-bus base; execute at the alias.
        self.mem.load_bytes(CODE_DBUS_BASE, code);
        let entry = Memory::ibus_alias(CODE_DBUS_BASE).wrapping_add(entry_offset);
        self.stage_windowed_entry(entry, arg);
        self.run_loop(tracer, None)
    }

    /// Run already-loaded code (e.g. ELF segments written into `self.mem` by a
    /// loader) starting at the I-bus address `entry`, invoked via the same
    /// synthesized windowed CALL8 as [`run`](Self::run). `SYSCALL` instructions
    /// are dispatched to `handler`.
    pub fn run_loaded(
        &mut self,
        entry: u32,
        arg: u32,
        tracer: &mut dyn Tracer,
        handler: &mut dyn SyscallHandler,
    ) -> RunOutcome {
        self.stage_windowed_entry(entry, arg);
        self.run_loop(tracer, Some(handler))
    }

    /// Reset the CPU and stage the synthesized windowed CALL8 into `entry`.
    fn stage_windowed_entry(&mut self, entry: u32, arg: u32) {
        // Synthesize the caller frame (the runner's context) at base 0 and the
        // CALL8 that jumps into `entry`. A real CALL8 writes the (mangled)
        // return address into the caller's a8 and stages args in a10..; the
        // callee's ENTRY then rotates WindowBase by PS.CALLINC (=2), so a8→a0
        // and a10→a2.
        self.cpu = Cpu::new();
        self.cpu.window_base = 0;
        self.cpu.window_start = 1; // frame 0 resident
        self.cpu.call_stack.push(crate::cpu::FrameRec {
            base: 0,
            sp: INITIAL_SP,
            inc: 2,
            resident: true,
        });
        self.cpu.set_a(1, INITIAL_SP); // caller SP
                                       // Mangled sentinel return address in a8: callinc=2 in top bits, sentinel
                                       // low bits. RETW unmangles to SENTINEL_PC (see finish_call).
        self.cpu.set_a(8, (2u32 << 30) | (SENTINEL_PC & 0x3FFF_FFFF));
        self.cpu.set_a(10, arg); // first argument
        self.cpu.ps_callinc = 2;
        self.cpu.pc = entry;
    }

    fn run_loop(
        &mut self,
        tracer: &mut dyn Tracer,
        mut handler: Option<&mut dyn SyscallHandler>,
    ) -> RunOutcome {
        let mut steps = 0u64;
        loop {
            if self.cpu.pc == SENTINEL_PC {
                // Top-level RETW landed on the sentinel: the result is in the
                // caller's a10 (== the callee's a2 before the return rotation).
                return RunOutcome::Ok(self.cpu.a(10));
            }
            if steps >= self.step_budget {
                return RunOutcome::Trap(Trap {
                    kind: TrapKind::Timeout,
                    cause: 0,
                    pc: self.cpu.pc,
                    vaddr: 0,
                });
            }
            steps += 1;
            match self.step(tracer) {
                Ok(Step::Normal) => {}
                Ok(Step::Syscall { next_pc }) => match handler.as_mut() {
                    // No handler: model unhandled hardware behavior (a
                    // SyscallCause exception at the SYSCALL's pc).
                    None => {
                        return RunOutcome::Trap(Trap {
                            kind: TrapKind::Exception,
                            cause: crate::error::EXC_SYSCALL,
                            pc: self.cpu.pc,
                            vaddr: 0,
                        })
                    }
                    Some(h) => match h.syscall(&mut self.cpu, &mut self.mem) {
                        SyscallOutcome::Resume(v) => {
                            self.cpu.set_a(2, v);
                            self.cpu.pc = next_pc;
                        }
                        SyscallOutcome::Exit(code) => return RunOutcome::Ok(code),
                    },
                },
                Err(mut trap) => {
                    if trap.pc == 0 {
                        trap.pc = self.cpu.pc;
                    }
                    return RunOutcome::Trap(trap);
                }
            }
        }
    }

    /// Fetch, decode, and execute one instruction, updating `pc`.
    fn step(&mut self, tracer: &mut dyn Tracer) -> Result<Step, Trap> {
        let pc = self.cpu.pc;
        let mut bytes = [0u8; 3];
        let got = self.mem.fetch(pc, &mut bytes)?;
        let (inst, len) = lp_xt_inst::decode(&bytes[..got]).map_err(|_| Trap {
            kind: TrapKind::Exception,
            cause: crate::error::EXC_ILLEGAL_INSTRUCTION,
            pc,
            vaddr: 0,
        })?;
        tracer.event(TraceEvent::Inst {
            pc,
            len,
            inst: &inst,
        });
        match self.execute(&inst, pc, tracer)? {
            Flow::Next => self.cpu.pc = pc.wrapping_add(len as u32),
            Flow::Jump(addr) => self.cpu.pc = addr,
            // Leave pc at the SYSCALL; the run loop advances after dispatch.
            Flow::Syscall => {
                return Ok(Step::Syscall {
                    next_pc: pc.wrapping_add(len as u32),
                })
            }
        }
        Ok(Step::Normal)
    }

    // --- small shared helpers used by the executor modules ---

    /// Write windowed register `a{i}` and emit a trace event.
    pub(crate) fn wreg(&mut self, i: u8, v: u32, tracer: &mut dyn Tracer) {
        let phys = self.cpu.set_a(i, v);
        tracer.event(TraceEvent::RegWrite {
            index: i,
            phys,
            value: v,
        });
    }

    /// Read windowed register `a{i}`.
    #[inline]
    pub(crate) fn rreg(&self, i: u8) -> u32 {
        self.cpu.a(i)
    }
}

impl Default for Emulator {
    fn default() -> Self {
        Emulator::new()
    }
}
