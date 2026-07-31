/// Features that replace `main`'s demo loop with a harness of their own.
///
/// Each is a whole build mode rather than a modifier, and they are mutually
/// exclusive (`src/main.rs` enforces that with `compile_error!`). Naming them
/// here lets the source say `#[cfg(demo_build)]` once instead of repeating a
/// growing `not(any(feature = …, feature = …))` list on thirty items.
const ALT_MAIN_FEATURES: [&str; 4] = ["TEST_LOOPBACK", "DIAG", "SWEEP_CHANNELS", "TEST_STRESS"];

fn main() {
    linker_be_nice();
    demo_build_cfg();
    // linkall.x must be the last linker script.
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

/// Emit `cfg(demo_build)` unless some feature has taken `main` over.
fn demo_build_cfg() {
    println!("cargo::rustc-check-cfg=cfg(demo_build)");
    for feature in ALT_MAIN_FEATURES {
        println!("cargo:rerun-if-env-changed=CARGO_FEATURE_{feature}");
    }
    let alt = ALT_MAIN_FEATURES
        .iter()
        .any(|f| std::env::var_os(format!("CARGO_FEATURE_{f}")).is_some());
    if !alt {
        println!("cargo::rustc-cfg=demo_build");
    }
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
