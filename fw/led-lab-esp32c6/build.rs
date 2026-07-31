fn main() {
    // linkall.x (from esp-hal) must be the last linker script.
    //
    // Unlike the S3 crate this does *not* also install esp-hal's
    // `--error-handling-script` hint: the RISC-V target links with `rust-lld`
    // directly rather than through a GCC driver, so the `-Wl,` prefix that
    // shape needs is rejected outright ("unknown argument").
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}
