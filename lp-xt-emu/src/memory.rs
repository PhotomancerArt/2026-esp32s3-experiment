//! Flat, `Vec`-backed memory with the ESP32-S3 SRAM1 D-bus / I-bus dual mapping.
//!
//! The hardware fact this models (see `../FINDINGS.md`, E2): a byte written at a
//! D-bus SRAM1 address (`0x3FC8_8000..0x3FCF_0000`) is fetchable at the I-bus
//! alias `+0x6F_0000`. The `xt-runner` firmware writes payloads via the D-bus
//! address and *executes them at the I-bus alias*, so self-addressing code
//! (`l32r` literals, `call8` targets) only behaves identically if the emulator
//! models the same alias — one backing store reachable at two address ranges.
//!
//! Original code; no derivation from QEMU/binutils (see the repo license ADR).

use crate::error::{Trap, TrapKind};

/// ESP32-S3 SRAM1 D-bus window start (data view).
pub const SRAM1_DBUS_START: u32 = 0x3FC8_8000;
/// ESP32-S3 SRAM1 D-bus window end (exclusive).
pub const SRAM1_DBUS_END: u32 = 0x3FCF_0000;
/// Offset from a D-bus SRAM1 address to its I-bus (instruction) alias.
pub const IBUS_ALIAS_OFFSET: u32 = 0x006F_0000;

/// A contiguous, `Vec`-backed memory region.
///
/// A region is addressable at its D-bus range `[dbus_start, dbus_start + len)`
/// for data access. If `alias_offset != 0` the same backing bytes are *also*
/// addressable — for both fetch and data — at the I-bus alias range
/// `[dbus_start + alias_offset, dbus_start + alias_offset + len)`.
struct Region {
    dbus_start: u32,
    alias_offset: u32,
    data: Vec<u8>,
    writable: bool,
}

impl Region {
    /// Byte index within `data` for `addr` if it falls in the D-bus range.
    fn dbus_index(&self, addr: u32) -> Option<usize> {
        let end = self.dbus_start.wrapping_add(self.data.len() as u32);
        if addr >= self.dbus_start && addr < end {
            Some((addr - self.dbus_start) as usize)
        } else {
            None
        }
    }

    /// Byte index within `data` for `addr` if it falls in the I-bus alias range.
    fn ibus_index(&self, addr: u32) -> Option<usize> {
        if self.alias_offset == 0 {
            return None;
        }
        let start = self.dbus_start.wrapping_add(self.alias_offset);
        let end = start.wrapping_add(self.data.len() as u32);
        if addr >= start && addr < end {
            Some((addr - start) as usize)
        } else {
            None
        }
    }
}

/// The emulator's physical address space: a set of regions plus the SRAM1 alias.
pub struct Memory {
    regions: Vec<Region>,
    /// Max number of bytes accessible from any address (for load/store bounds).
    #[allow(dead_code)]
    _reserved: (),
}

/// How a resolved address may be used.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Access {
    /// Data load/store — permitted at either the D-bus or I-bus view.
    Data,
    /// Instruction fetch — permitted only at the I-bus (executable) view. A
    /// fetch of a D-bus-only address models the hardware `InstrFetchError`
    /// (FINDINGS E2D: jumping to the D-bus address faults, EXCCAUSE 2).
    Fetch,
}

impl Memory {
    /// Empty address space.
    pub fn new() -> Memory {
        Memory {
            regions: Vec::new(),
            _reserved: (),
        }
    }

    /// Add a plain read/write data region with no executable alias.
    pub fn add_ram(&mut self, dbus_start: u32, len: usize) {
        self.regions.push(Region {
            dbus_start,
            alias_offset: 0,
            data: vec![0u8; len],
            writable: true,
        });
    }

    /// Add a region backing the SRAM1 dual mapping: writable via the D-bus
    /// range and fetchable/readable via the I-bus alias `+0x6F_0000`.
    pub fn add_sram1(&mut self, dbus_start: u32, len: usize) {
        assert!(
            (SRAM1_DBUS_START..SRAM1_DBUS_END).contains(&dbus_start),
            "SRAM1 region base {dbus_start:#x} outside the dual-mapped window"
        );
        self.regions.push(Region {
            dbus_start,
            alias_offset: IBUS_ALIAS_OFFSET,
            data: vec![0u8; len],
            writable: true,
        });
    }

    /// The I-bus (executable) alias of a D-bus SRAM1 address.
    pub fn ibus_alias(dbus_addr: u32) -> u32 {
        dbus_addr.wrapping_add(IBUS_ALIAS_OFFSET)
    }

    /// Resolve `addr` for `access`, returning `(region_index, byte_index)`.
    fn resolve(&self, addr: u32, access: Access) -> Option<(usize, usize)> {
        // Fetch: only the executable (I-bus alias) view is valid.
        for (ri, r) in self.regions.iter().enumerate() {
            if let Some(idx) = r.ibus_index(addr) {
                return Some((ri, idx));
            }
            if access == Access::Data {
                if let Some(idx) = r.dbus_index(addr) {
                    return Some((ri, idx));
                }
            }
        }
        None
    }

    /// Copy `bytes` into the region covering `dbus_addr` (data write, ignores
    /// the writable flag — this is loader setup, not guest execution).
    pub fn load_bytes(&mut self, dbus_addr: u32, bytes: &[u8]) {
        let (ri, idx) = self
            .resolve(dbus_addr, Access::Data)
            .unwrap_or_else(|| panic!("load_bytes: address {dbus_addr:#x} not mapped"));
        let r = &mut self.regions[ri];
        assert!(
            idx + bytes.len() <= r.data.len(),
            "load_bytes: {} bytes at {dbus_addr:#x} overruns region",
            bytes.len()
        );
        r.data[idx..idx + bytes.len()].copy_from_slice(bytes);
    }

    // --- fetch ---

    /// Read up to `n` (1..=3) instruction bytes for decode, honoring fetch
    /// permission. Returns fewer bytes only at the very end of a region.
    pub fn fetch(&self, pc: u32, out: &mut [u8; 3]) -> Result<usize, Trap> {
        // The first byte must be fetchable; that classifies the address.
        if self.resolve(pc, Access::Fetch).is_none() {
            return Err(Trap {
                kind: TrapKind::Exception,
                cause: EXC_INSTR_FETCH_ERROR,
                pc,
                vaddr: pc,
            });
        }
        let mut got = 0;
        for i in 0..3u32 {
            match self.resolve(pc.wrapping_add(i), Access::Fetch) {
                Some((ri, idx)) => {
                    out[i as usize] = self.regions[ri].data[idx];
                    got += 1;
                }
                None => break,
            }
        }
        Ok(got)
    }

    // --- typed data access ---

    fn read_bytes(&self, addr: u32, n: u32) -> Result<u32, Trap> {
        let mut v = 0u32;
        for i in 0..n {
            let a = addr.wrapping_add(i);
            match self.resolve(a, Access::Data) {
                Some((ri, idx)) => v |= (self.regions[ri].data[idx] as u32) << (8 * i),
                None => return Err(self.load_fault(addr)),
            }
        }
        Ok(v)
    }

    fn write_bytes(&mut self, addr: u32, n: u32, val: u32) -> Result<(), Trap> {
        for i in 0..n {
            let a = addr.wrapping_add(i);
            match self.resolve(a, Access::Data) {
                Some((ri, idx)) => {
                    if !self.regions[ri].writable {
                        return Err(self.store_fault(addr));
                    }
                    self.regions[ri].data[idx] = (val >> (8 * i)) as u8;
                }
                None => return Err(self.store_fault(addr)),
            }
        }
        Ok(())
    }

    pub fn read_u8(&self, addr: u32) -> Result<u8, Trap> {
        Ok(self.read_bytes(addr, 1)? as u8)
    }
    pub fn read_u16(&self, addr: u32) -> Result<u16, Trap> {
        Ok(self.read_bytes(addr, 2)? as u16)
    }
    pub fn read_u32(&self, addr: u32) -> Result<u32, Trap> {
        self.read_bytes(addr, 4)
    }
    pub fn write_u8(&mut self, addr: u32, v: u8) -> Result<(), Trap> {
        self.write_bytes(addr, 1, v as u32)
    }
    pub fn write_u16(&mut self, addr: u32, v: u16) -> Result<(), Trap> {
        self.write_bytes(addr, 2, v as u32)
    }
    pub fn write_u32(&mut self, addr: u32, v: u32) -> Result<(), Trap> {
        self.write_bytes(addr, 4, v)
    }

    fn load_fault(&self, addr: u32) -> Trap {
        Trap {
            kind: TrapKind::Exception,
            cause: EXC_LOAD_STORE_ERROR,
            pc: 0,
            vaddr: addr,
        }
    }
    fn store_fault(&self, addr: u32) -> Trap {
        Trap {
            kind: TrapKind::Exception,
            cause: EXC_LOAD_STORE_ERROR,
            pc: 0,
            vaddr: addr,
        }
    }
}

impl Default for Memory {
    fn default() -> Self {
        Memory::new()
    }
}

/// EXCCAUSE for an instruction fetch to a non-executable address (matches the
/// S3's `InstrFetchError`; FINDINGS E2D observed EXCCAUSE 2).
pub const EXC_INSTR_FETCH_ERROR: u32 = 2;
/// EXCCAUSE for a bad load/store address (`LoadStoreErrorCause`).
pub const EXC_LOAD_STORE_ERROR: u32 = 3;
