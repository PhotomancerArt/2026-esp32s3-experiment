//! Heap-allocated executable code buffers (from the spike; see FINDINGS.md).
//!
//! ESP32-S3 internal SRAM1 is dual-mapped: a byte written at a D-bus address
//! (`0x3FC8_8000..0x3FCF_0000`) is fetchable at the I-bus alias `+0x6F_0000`.
//! esp-alloc's heap lives in dram_seg (SRAM1), so heap allocations have an
//! executable alias; the alias math asserts we never hand out a non-executable
//! pointer.

use alloc::alloc::{alloc, dealloc};
use core::alloc::Layout;

pub const SRAM1_DBUS_START: usize = 0x3FC8_8000;
pub const SRAM1_DBUS_END: usize = 0x3FCF_0000;
pub const IBUS_ALIAS_OFFSET: usize = 0x006F_0000;

/// A 4-byte-aligned heap buffer holding machine code.
pub struct JitBuf {
    ptr: *mut u8,
    layout: Layout,
}

impl JitBuf {
    pub fn new(bytes: &[u8]) -> JitBuf {
        let layout = Layout::from_size_align(bytes.len().max(4), 4).expect("layout");
        // SAFETY: layout has non-zero size; pointer checked below.
        let ptr = unsafe { alloc(layout) };
        assert!(!ptr.is_null(), "jitbuf alloc failed");
        // SAFETY: ptr valid for layout.size() >= bytes.len() writes.
        unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), ptr, bytes.len()) };
        JitBuf { ptr, layout }
    }

    /// The I-bus alias — the address to execute from.
    pub fn exec_addr(&self) -> usize {
        let d = self.ptr as usize;
        assert!(
            (SRAM1_DBUS_START..SRAM1_DBUS_END).contains(&d),
            "jitbuf at {d:#x} outside dual-mapped SRAM1"
        );
        d + IBUS_ALIAS_OFFSET
    }
}

impl Drop for JitBuf {
    fn drop(&mut self) {
        // SAFETY: ptr/layout came from the successful alloc in `new`.
        unsafe { dealloc(self.ptr, self.layout) };
    }
}

/// Instruction-fetch + memory barriers after writing code, before executing it.
pub fn sync_code() {
    // SAFETY: memw/isync have no operands and no memory-safety impact.
    unsafe {
        core::arch::asm!("memw");
        core::arch::asm!("isync");
    }
}
