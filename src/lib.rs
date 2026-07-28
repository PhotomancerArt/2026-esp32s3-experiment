#![no_std]
// Xtensa inline/global asm is not stable even on the esp fork.
#![feature(asm_experimental_arch)]

extern crate alloc;

pub mod e2;
pub mod jitbuf;
