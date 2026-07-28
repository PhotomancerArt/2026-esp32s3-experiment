//! Heap-allocated buffers for dynamically generated code.
//!
//! ESP32-S3 internal SRAM1 is dual-mapped: the same physical byte is readable/
//! writable at a D-bus address (`0x3FC8_8000..0x3FCF_0000`) and fetchable at an
//! I-bus address (`0x4037_8000..0x403E_0000`), a fixed `+0x6F_0000` alias.
//! (Source: esp-hal `ld/esp32s3/memory.x` + ESP32-S3 TRM ch. "System and Memory".)
//! The esp-alloc heap is a static in dram_seg, so heap allocations land in SRAM1
//! and have an executable alias. SRAM2 (`0x3FCF_0000..`) has no I-bus alias —
//! the alias math asserts we never hand out an unexecutable pointer.

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
    /// Allocate and fill a code buffer from `bytes`.
    pub fn new(bytes: &[u8]) -> JitBuf {
        let layout = Layout::from_size_align(bytes.len().max(4), 4).expect("layout");
        // SAFETY: layout has non-zero size; we check the returned pointer.
        let ptr = unsafe { alloc(layout) };
        assert!(!ptr.is_null(), "jitbuf alloc failed");
        // SAFETY: ptr is valid for layout.size() >= bytes.len() writes.
        unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), ptr, bytes.len()) };
        JitBuf { ptr, layout }
    }

    pub fn write_addr(&self) -> usize {
        self.ptr as usize
    }

    /// The I-bus alias of this buffer — the address to jump to.
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
        // SAFETY: ptr/layout come from the successful alloc in `new`.
        unsafe { dealloc(self.ptr, self.layout) };
    }
}

/// Instruction-fetch pipeline barrier.
pub fn isync() {
    // SAFETY: isync has no operands and no memory safety impact.
    unsafe { core::arch::asm!("isync") };
}

/// Memory-ordering barrier.
pub fn memw() {
    // SAFETY: memw has no operands and no memory safety impact.
    unsafe { core::arch::asm!("memw") };
}
