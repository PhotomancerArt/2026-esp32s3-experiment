fn main() {
    linker_be_nice();
    // linkall.x must be the last linker script.
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        if kind.as_str() == "undefined-symbol" {
            if what.as_str() == "_stack_start" {
                eprintln!();
                eprintln!("💡 Is the linker script `linkall.x` missing?");
                eprintln!();
            }
        } else {
            // Nothing helpful to add for other link errors (e.g. missing-lib).
            std::process::exit(1);
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=-Wl,--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
